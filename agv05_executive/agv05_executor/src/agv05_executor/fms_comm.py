from __future__ import absolute_import
from __future__ import unicode_literals

from kombu import Connection, Consumer
from kombu.asynchronous import Hub, set_event_loop
from kombu.pools import connections, producers, set_limit
import amqp.transport
import contextlib
import rospy


__all__ = ['FmsComm', 'get_connection', 'get_producer']


# Monkey-patch py-amqp library

CONNECT_TIMEOUT = 3
TRANSFER_TIMEOUT = 7


def wrap_setup_transport(Transport):
    __old_setup_transport = Transport._setup_transport

    def _setup_transport(self):
        self.sock.settimeout(TRANSFER_TIMEOUT)
        __old_setup_transport(self)

    Transport._setup_transport = _setup_transport


wrap_setup_transport(amqp.transport.SSLTransport)
wrap_setup_transport(amqp.transport.TCPTransport)


set_limit(1)


def set_connection_url(url):
    global _conn
    _conn = FmsConnection(url, connect_timeout=CONNECT_TIMEOUT, ssl=(':5671/' in url))


@contextlib.contextmanager
def get_connection():
    with connections[_conn].acquire(block=True) as connection:
        yield connection


@contextlib.contextmanager
def get_producer():
    with producers[_conn].acquire(block=True) as producer:
        yield producer


class FmsConnection(Connection):

    def __init__(self, *args, **kwargs):
        super(FmsConnection, self).__init__(*args, **kwargs)
        self.transport_options = {
            'max_retries': len(self.alt) - 1 or -1,  # -1 = no retry
        }
        self._auto_consumers = set()

    def AutoConsumer(self, *args, **kwargs):
        # ignore decode error
        kwargs['on_decode_error'] = lambda msg, exc: None

        try:
            consumer = self.Consumer(*args, **kwargs)
        except Exception:
            consumer = Consumer(None, *args, **kwargs)
        self.__register_auto_consumer(consumer)
        consumer._orig_cancel = consumer.cancel
        consumer.cancel = lambda: self.__unregister_auto_consumer(consumer)
        return consumer

    def __register_auto_consumer(self, consumer):
        self._auto_consumers.add(consumer)
        if self.connected:
            try:
                consumer.consume()
            except Exception:
                pass

    def __unregister_auto_consumer(self, consumer):
        self._auto_consumers.discard(consumer)
        if self.connected:
            try:
                consumer._orig_cancel()
            except Exception:
                pass

    def reconnect(self):
        self.close()
        self.ensure_connection(**self.transport_options)
        for consumer in self._auto_consumers:
            consumer.revive(self)
            consumer.consume()


class FmsComm(object):
    recoverable_connection_errors = []

    def __init__(self, broker):
        self.hub = set_event_loop(Hub())
        set_connection_url(broker)

        with get_connection() as connection:
            self.recoverable_connection_errors = connection.connection_errors + connection.recoverable_connection_errors
            # allow hub's timer to propagate these errors, which is otherwise suppressed.
            self.hub.propagate_errors = self.recoverable_connection_errors

        self.revive()

    def revive(self):
        try:
            with get_connection() as connection:
                # NOTE: If we increase pool limit to be greater than 1,
                # we must register every new connection from the pool
                connection.reconnect()
                connection.register_with_event_loop(self.hub)
        except Exception as ex:
            rospy.logwarn('[Fms Comm] Connection open error. Reconnecting in 1 second: %s', ex)
            self.hub.call_later(1.0, self.revive)
        else:
            rospy.loginfo('Connection opened.')

    def spin(self):
        while True:
            try:
                self.hub.run_forever()
            except self.recoverable_connection_errors as ex:
                rospy.logwarn('[Fms Comm]: Connection lost. Reconnecting...: %s', ex)
                self.hub.call_soon(self.revive)
            finally:
                try:
                    self.hub.reset()
                except Exception as ex:
                    rospy.logerr('[Fms Comm]: Error cleaning up after event loop: %r', ex, exc_info=1)
