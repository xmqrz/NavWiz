from __future__ import absolute_import
from __future__ import unicode_literals

import threading


# from: https://www.oreilly.com/library/view/python-cookbook/0596001673/ch06s04.html
class ReadWriteLock(object):
    """ A lock object that allows many simultaneous "read locks", but
    only one "write lock." """

    def __init__(self):
        self._read_ready = threading.Condition(threading.Lock())
        self._readers = 0

        self.read = ContextHelper(
            lambda: self.acquire_read(),
            lambda: self.release_read(),
        )
        self.write = ContextHelper(
            lambda: self.acquire_write(),
            lambda: self.release_write(),
        )

    def acquire_read(self):
        """ Acquire a read lock. Blocks only if a thread has
        acquired the write lock. """
        self._read_ready.acquire()
        try:
            self._readers += 1
        finally:
            self._read_ready.release()

    def release_read(self):
        """ Release a read lock. """
        self._read_ready.acquire()
        try:
            self._readers -= 1
            if not self._readers:
                self._read_ready.notifyAll()
        finally:
            self._read_ready.release()

    def acquire_write(self):
        """ Acquire a write lock. Blocks until there are no
        acquired read or write locks. """
        self._read_ready.acquire()
        while self._readers > 0:
            self._read_ready.wait()

    def release_write(self):
        """ Release a write lock. """
        self._read_ready.release()


class ContextHelper(object):
    def __init__(self, aquire, release):
        self.aquire = aquire
        self.release = release

    def __enter__(self):
        self.aquire()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
