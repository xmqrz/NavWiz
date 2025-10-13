from __future__ import absolute_import
from __future__ import unicode_literals

import hashlib
import logging
import ujson as json

logger = logging.getLogger(__name__)


class ValidationError(RuntimeError):
    def __init__(self, *args, **kwargs):
        self.params = kwargs.pop('params', {})
        super(ValidationError, self).__init__(*args, **kwargs)

    @property
    def error_msg(self):
        return str(self).format(**{k: v['name'] for k, v in self.params.items()})


class Validator(object):
    cache = None
    provides = []
    extra_provides = []

    def __init__(self, models):
        if not self.cache:
            raise RuntimeError('The cache object must be defined in Validator\'s subclass.')

        self.valid = False
        self.models = models
        self.robot = models.robot
        for p in self.provides + self.extra_provides:
            setattr(self.models, p, None)

    def is_enabled(self, pre_validate=False):
        # overwrite to disable validator
        return True

    def validate(self):
        # validate and build provides (including extras)
        raise NotImplementedError()

    def assemble(self):
        # assemble extras from cache
        pass

    def validate_and_share_tmp(self):
        self.valid = False
        if not self.is_enabled(pre_validate=True):
            return

        self.validate()
        for p in self.provides + self.extra_provides:
            self.models.tmp[p] = getattr(self, p)
        self.valid = True

    def is_valid(self):
        return self.valid

    def apply(self):
        if not self.is_enabled():
            return

        if not self.is_valid():
            raise RuntimeError('Trying to apply invalid data in Validator.')

        for p in self.provides + self.extra_provides:
            setattr(self.models, p, getattr(self, p))

        for p in self.provides:
            self._set_cache(p, getattr(self, p))

    def hot_reload(self, downloadables=None):
        # override to allow reload action
        pass

    def from_cache(self):
        if not self.is_enabled():
            return

        for p in self.provides:
            setattr(self, p, self._get_cache(p))

        self.assemble()

        for p in self.provides + self.extra_provides:
            setattr(self.models, p, getattr(self, p))

    def _get_cache(self, key):
        return getattr(self.cache, 'get_' + key)()

    def _set_cache(self, key, value):
        return getattr(self.cache, 'set_' + key)(value)

    def from_downloadables(self, downloadables):
        assert self.models.is_fms_mode()
        if self.is_enabled():
            # run only validators of standalone mode, which have not run previously.
            return

        for p in self.provides:
            setattr(self, p, downloadables.get(p))

        self.assemble()

        for p in self.provides + self.extra_provides:
            setattr(self.models, p, getattr(self, p))

        for p in self.provides:
            self._set_cache(p, getattr(self, p))

    def check_reloadable(self, data):
        for p in self.provides:
            cur = getattr(self.models, p, None)
            if cur is None:
                return False
            tmp = data[p]
            try:
                if not self.provide_reloadable(p, cur, tmp):
                    return False
            except Exception as e:
                logger.exception(
                    'Checking provide reloadable failed for "%s":"%s"'
                    % (type(self), p)
                )
                return False
        return True

    def provide_reloadable(self, provide, cur, tmp):
        def hash(data):
            return hashlib.md5(json.dumps(data, sort_keys=True).encode('utf-8')).hexdigest()
        return hash(cur) == hash(tmp)
