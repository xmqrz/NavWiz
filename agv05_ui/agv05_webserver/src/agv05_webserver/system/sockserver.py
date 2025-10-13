from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.system.models import db_auto_reconnect
from django.conf import settings as django_settings
from django.contrib.auth import get_user, get_user_model
from functools import partial
from importlib import import_module
from rest_framework.authtoken.models import Token
from sockjs.tornado import SockJSConnection
from tornado.ioloop import IOLoop
import logging
import ujson as json

from .channels import ChannelManager

User = get_user_model()

logger = logging.getLogger(__name__)


class AuthManager(object):
    # Obtain the user who is on the websocket connection.
    # See http://developers-club.com/posts/128562/
    _engine = import_module(django_settings.SESSION_ENGINE)

    def _get_session(self, session_key):
        return self._engine.SessionStore(session_key)

    def _get_user(self, session):
        class Dummy(object):
            pass

        django_request = Dummy()
        django_request.session = session

        return get_user(django_request)

    def _set_user(self, connection, user):
        if user.is_anonymous:
            user = User.objects.get(username='Guest')

        connection.user = user
        connection.perms = user.get_all_permissions()

        if user.username == 'agv_panel':
            connection.user_with_pin = User.objects.get(username='agv_panel_pin_protected')
            connection.perms_with_pin = connection.user_with_pin.get_all_permissions()
        else:
            connection.user_with_pin = None
            connection.perms_with_pin = set()

    @db_auto_reconnect()
    def setup_user(self, connection, info):
        session_key = None
        try:
            session_key = info.get_cookie(django_settings.SESSION_COOKIE_NAME).value
        except Exception:
            pass

        connection.django_session = self._get_session(session_key)
        self._set_user(connection, self._get_user(connection.django_session))

    @db_auto_reconnect()
    def setup_token_user(self, connection, key):
        token = Token.objects.select_related('user').filter(key=key).first()
        if token:
            self._set_user(connection, token.user)


class Connection(SockJSConnection):
    auth_manager = None
    channel_manager = None
    channel_manager_cls = ChannelManager
    clients = set()
    last_conn_id = 0

    def on_open(self, info):
        if not Connection.auth_manager:
            Connection.auth_manager = AuthManager()

        if not Connection.channel_manager:
            Connection.channel_manager = self.channel_manager_cls()
            Connection.channel_manager.clients = self.clients

        # Comment this since it may clog the websocket and it is unused for now.
        # self.auth_manager.setup_user(self, info)
        self.clients.add(self)
        self.id = self._gen_id()
        logger.info('Client connected (id=%d). Total clients: %d', self.id, len(self.clients))

    def on_message(self, msg):
        try:
            msg = json.loads(msg)
            if msg['action'] == 'subscribe':
                self.channel_manager.subscribe(msg['channel'], self)

            elif msg['action'] == 'unsubscribe':
                self.channel_manager.unsubscribe(msg['channel'], self)

            elif msg['action'] == 'publish':
                self.channel_manager.publish(msg['channel'], msg['data'], self)

        except (KeyError, ValueError):
            logger.warning('Client message format error.')

    def on_close(self):
        self.channel_manager.unsubscribe_all(self)
        self.clients.remove(self)
        logger.info('Client disconnected. Total clients: %d', len(self.clients))

    # thread-safe broadcast and send methods
    def broadcast(self, clients, msg):
        IOLoop.instance().add_callback(partial(super(Connection, self).broadcast, clients, msg))

    def send(self, msg):
        IOLoop.instance().add_callback(partial(super(Connection, self).send, msg))

    def _gen_id(self):
        Connection.last_conn_id += 1
        return self.last_conn_id

    @classmethod
    def get_client_count(cls):
        return len(cls.clients)
