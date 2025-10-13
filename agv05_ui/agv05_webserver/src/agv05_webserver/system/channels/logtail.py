from __future__ import absolute_import
from __future__ import unicode_literals

from tornado.process import Subprocess
import collections
import os
import signal


class Logtail(object):

    def __init__(self, filename, lines=100):
        self.filename = filename
        self.__lines = collections.deque(maxlen=lines)
        self.__proc = None

    def iter_lines(self):
        return self.__lines

    def start(self, callback):
        if self.__proc:
            return

        self.callback = callback if callback and callable(callback) else None
        self.__proc = Subprocess(
            ['tail', '-F', self.filename, '-n', '%d' % self.__lines.maxlen],
            stdout=Subprocess.STREAM,
            universal_newlines=True,
            bufsize=1)  # line buffered
        self.__proc.stdout.read_until(b'\n', self.__handle_log)

    def __handle_log(self, line):
        line = line.decode()
        self.__lines.append(line)
        self.callback(line)
        if self.__proc:
            self.__proc.stdout.read_until(b'\n', self.__handle_log)

    def stop(self):
        if not self.__proc:
            return

        self.__proc.stdout.close()
        os.kill(self.__proc.pid, signal.SIGTERM)
        self.__proc = None
        self.__lines.clear()
