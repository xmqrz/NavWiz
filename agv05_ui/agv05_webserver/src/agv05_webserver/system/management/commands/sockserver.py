from __future__ import absolute_import
from __future__ import unicode_literals

from datetime import datetime
from django.conf import settings as django_settings
from django.core.exceptions import ImproperlyConfigured
from django.core.management.base import BaseCommand, CommandError
from django.utils import six
from django.utils.encoding import force_text, get_system_encoding
from importlib import import_module
from sockjs.tornado import SockJSRouter
from tornado import web, ioloop
import errno
import re
import socket
import sys


naiveip_re = re.compile(r"""^(?:
(?P<addr>
    (?P<ipv4>\d{1,3}(?:\.\d{1,3}){3}) |         # IPv4 address
    (?P<ipv6>\[[a-fA-F0-9:]+\]) |               # IPv6 address
    (?P<fqdn>[a-zA-Z0-9-]+(?:\.[a-zA-Z0-9-]+)*) # FQDN
):)?(?P<port>\d+)$""", re.X)


class Command(BaseCommand):
    help = 'Run the sockjs-tornado server.'
    default_port = 9000

    def add_arguments(self, parser):
        parser.add_argument('addrport', nargs='?',
            help='Optional port number, or ipaddr:port')
        parser.add_argument('--ipv6', '-6', action='store_true', dest='use_ipv6', default=False,
            help='Tells Django to use an IPv6 address.')
        parser.add_argument('--autoreload', action='store_true', dest='autoreload', default=False,
            help='Enables the auto-reloader for tornado.')
        parser.add_argument('--no-keep-alive', action='store_true', dest='no_keep_alive', default=False,
            help='Sets no_keep_alive on the connection if your server needs it.')

    def handle(self, *args, **options):
        self.use_ipv6 = options.get('use_ipv6')
        if self.use_ipv6 and not socket.has_ipv6:
            raise CommandError('Your Python does not support IPv6.')
        self._raw_ipv6 = False
        if not options.get('addrport'):
            self.addr = ''
            self.port = self.default_port
        else:
            m = re.match(naiveip_re, options['addrport'])
            if m is None:
                raise CommandError('"%s" is not a valid port number '
                                   'or address:port pair.' % options['addrport'])
            self.addr, _ipv4, _ipv6, _fqdn, self.port = m.groups()
            if not self.port.isdigit():
                raise CommandError("%r is not a valid port number." % self.port)
            if self.addr:
                if _ipv6:
                    self.addr = self.addr[1:-1]
                    self.use_ipv6 = True
                    self._raw_ipv6 = True
                elif self.use_ipv6 and not _fqdn:
                    raise CommandError('"%s" is not a valid IPv6 address.' % self.addr)
        if not self.addr:
            self.addr = '::1' if self.use_ipv6 else '127.0.0.1'
            self._raw_ipv6 = bool(self.use_ipv6)
        self.run(**options)

    def run(self, **options):
        connections = getattr(django_settings, 'SOCKJS_CONNECTIONS', {})

        if len(connections) == 0:
            raise ImproperlyConfigured('Must define at least one connection in the SOCKJS_CONNECTIONS settings.')

        if not isinstance(connections, dict):
            raise ImproperlyConfigured('SOCKJS_CONNECTIONS settings should be a dictionary containing the url prefix and the corresponding connection class.')

        routers = []
        try:
            for prefix, conn_class in six.iteritems(connections):
                module_name, cls_name = conn_class.rsplit('.', 1)
                module = import_module(module_name)
                cls = getattr(module, cls_name)
                if not prefix.startswith('/'):
                    prefix = '/%s' % prefix
                routers.append(SockJSRouter(cls, prefix))
        except Exception as e:
            raise ImproperlyConfigured('Failed to load configured class in SOCKJS_CONNECTIONS: %s' % force_text(e))

        app = web.Application([url for router in routers for url in router.urls], **{
            'autoreload': options.get('autoreload'),
            'debug': django_settings.DEBUG,
        })

        now = datetime.now().strftime('%B %d, %Y - %X')
        if six.PY2:
            now = now.decode(get_system_encoding())
        self.stdout.write(now)
        self.stdout.write('Starting sockjs-tornado server at ws://%s:%s/\n' % (self.addr, self.port))

        try:
            app.listen(self.port, self.addr, **{
                'no_keep_alive': options.get('no_keep_alive')
            })
            ioloop.IOLoop.instance().start()
        except socket.error as e:
            # Use helpful error messages instead of ugly tracebacks.
            ERRORS = {
                errno.EACCES: "You don't have permission to access that port.",
                errno.EADDRINUSE: "That port is already in use.",
                errno.EADDRNOTAVAIL: "That IP address can't be assigned to.",
            }
            try:
                error_text = ERRORS[e.errno]
            except KeyError:
                error_text = force_text(e)
            self.stderr.write('Error: %s' % error_text)
            sys.exit(1)
        except KeyboardInterrupt:
            ioloop.IOLoop.instance().stop()
            self.stdout.write('Shutting down server.')
            sys.exit(0)
