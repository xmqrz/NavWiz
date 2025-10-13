from __future__ import unicode_literals

from Cryptodome.Cipher import AES
from Cryptodome.Hash import SHA256
from Cryptodome.PublicKey import ECC
from Cryptodome.Random import get_random_bytes
from Cryptodome.Signature import eddsa
from collections import namedtuple
from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.contrib.auth.models import Group
from django.core.cache import cache, caches
from django.db import close_old_connections, models, transaction
from django.utils import timezone
from enumfields import IntEnum
from six import python_2_unicode_compatible
import base64
import contextdecorator
import inspect
import os
import pyotp
import six
import struct
import subprocess
import threading
import time
import ujson as json
import uuid

from .cpuid import CPUID


# WARNING: This creates a redis_cache instance which is shared by all threads.
# It is against Django's way to retrieve directly from `caches` everytime on use,
# so that one instance will be created per thread and not shared among threads.
# In our case, the DatabaseCache and RedisCache backends are thread-safe, but others might not.
redis_cache = caches['redis']

cpuid = CPUID()
K = b''.join([six.int2byte(c + i * 0x49 & 0xff) for i, c in enumerate(six.iterbytes(b'@$%bu}@Zj{ih75Je6+:Ta54CtJ@veXHO'))])
R = namedtuple('r', ['valid', 'owner', 'email', 'features', 'days', 'till'])
R0 = R(False, '', '', 0, -1, 0)

F_EE = 1 << 0  # enterprise edition (paid)
F_HW = 1 << 1  # run on hardware
F_VM = 1 << 2  # run on VM
F_MG = 1 << 16  # tracked mode
F_TS = 1 << 17  # trackless mode
F_OA = 1 << 18  # dynamic obstacle avoidance


# Create your models here.

class db_auto_reconnect(contextdecorator.ContextDecorator):
    reentrant = threading.local()

    def __enter__(self):
        if not hasattr(self.reentrant, 'state'):
            self.reentrant.state = False

        assert not self.reentrant.state  # Do not allow reentrant.
        self.reentrant.state = True
        close_old_connections()

    def __exit__(self, *exc):
        close_old_connections()
        self.reentrant.state = False


# shorten name to obfuscate
def myfp():
    # compute device fingerprint
    fp = {}
    d = _rf('/etc/machine-id')
    d1 = list(map(_pread, ['product_uuid', 'product_serial', 'board_serial', 'chassis_serial']))
    d2 = _rf('/sys/class/dmi/id/uevent')
    fp['machine'] = [d, d1, d2]

    fp['vm'] = bool(cpuid(0)[0] >= 1 and cpuid(1)[2] & (1 << 31))

    start = [0x0, 0x80000000]
    if fp['vm']:
        start.append(0x40000000)
    d = []
    for eax in start:
        r = cpuid(eax)
        highest = r[0]
        while True:
            d.append('%08x' * 4 % r)
            eax += 1
            if eax > highest:
                break
            r = cpuid(eax)
    fp['cpu'] = d

    try:
        with open('/proc/meminfo') as f:
            for line in f:
                d = line.split()
                if d[0] == 'MemTotal:':
                    m = int(d[1])
                    break
    except Exception:
        m = None
    fp['memory'] = m

    d = os.stat('/var').st_dev
    p = '/sys/dev/block/%s:%s' % (os.major(d), os.minor(d))
    p, partition = os.path.split(os.readlink(p))
    device = os.path.basename(p)

    p = '/sys/block'
    d1 = None
    d2 = []
    d3 = []
    for d in os.listdir(p):
        w = _rf(os.path.join(p, d, 'device/wwid'))
        if d == device:
            d1 = w
        elif w:
            d2.append(w)
        else:
            continue

        s = _rf(os.path.join(p, d, 'size'))
        try:
            s = int(s) * 512
        except Exception:
            s = 0
        if d == device:
            d3.insert(0, s)
        else:
            d3.append(s)

    p = '/dev/disk/by-uuid'
    d4 = None
    d5 = []
    for w in os.listdir(p):
        d = os.path.basename(os.readlink(os.path.join(p, w)))
        if d == partition:
            d4 = w
        else:
            d5.append(w)
    fp['disk'] = [[device, d1], d2, d3, [partition, d4], d5]

    p = '/sys/class/net'
    d1 = []
    for d in os.listdir(p):
        if not os.path.exists(os.path.join(p, d, 'device')):
            continue  # virtual
        if d.startswith('can') or d.startswith('rename'):
            continue
        d = _rf(os.path.join(p, d, 'address'))
        if d:
            d1.append(d)
    fp['network'] = d1

    try:
        obj = Variable.shared_qs().get(name=Variable.LICENSE_FP)
        assert timezone.now() + timezone.timedelta(days=1) > obj.modified  # allow for clock skew
        ev0 = obj.value
    except Exception:
        ev0 = None

    while True:
        try:
            assert ev0.startswith('NAVWIZ_FP')
            p = base64.b64decode(ev0[9:])
            ciphertext, tag, nonce = struct.unpack('%ss16s12s' % (len(p) - 28), p)
            fp0 = json.loads(AES.new(K, AES.MODE_GCM, nonce).decrypt_and_verify(ciphertext, tag))
        except Exception:
            nonce = get_random_bytes(12)
        else:
            # allow slight change in device specs
            d = fp['disk']
            d0 = fp0['disk']
            if (fp['machine'] == fp0['machine'] and
                    fp['vm'] == fp0['vm'] and
                    len(fp['cpu']) == len(fp0['cpu']) and
                    sum(sum(1 for d3, d4 in zip(d1, d2) if d3 != d4)
                        for d1, d2 in zip(fp['cpu'], fp0['cpu'])) < max(0x10, len(fp['cpu'])) and
                    d[0] == d0[0] and d[3] == d0[3] and
                    _pmatch(d[1], d0[1], 1) and
                    _pmatch(d[4], d0[4], 1) and
                    _pmatch(fp['network'], fp0['network'])):
                ev = ev0
                break

        cipher = AES.new(K, AES.MODE_GCM, nonce=nonce)
        ciphertext, tag = cipher.encrypt_and_digest(json.dumps(fp, sort_keys=True).encode('utf-8'))
        ev = 'NAVWIZ_FP' + base64.b64encode(b'%s%s%s' % (ciphertext, tag, nonce)).decode()

        if ev0:
            with Variable.shared_atomic():
                obj = Variable.shared_qs().select_for_update().filter(name=Variable.LICENSE_FP).first()
                if obj:
                    if obj.value == ev0:
                        obj.value = ev
                        obj.save()
                        break
                    else:
                        ev0 = obj.value
                else:
                    ev0 = None
        if not ev0:
            obj, created = Variable.shared_qs().get_or_create(name=Variable.LICENSE_FP, defaults={'value': ev})
            if created:
                break
            ev0 = obj.value

    return ev, _hash(ev)


def _pread(p):  # privileged read
    p = os.path.join('/sys/class/dmi/id', p)
    if os.stat(p).st_mode & 0x04 == 0:
        subprocess.check_call(['sudo', '-n', 'chmod', '0444', p])
    return _rf(p)


def _rf(p):  # read file
    try:
        with open(p, 'rb') as f:
            return f.read().strip().decode('latin')
    except Exception:
        pass


def _pmatch(d, d0, c=0):  # partial match
    n = len(d) + c
    n0 = len(d0) + c
    nc = len(set(d) & set(d0)) + c
    return n <= n0 * 2 and n0 <= n * 2 and max(n, n0) <= nc * 2


def _hash(k):
    return '%s' % uuid.UUID(bytes=SHA256.new(k.encode('utf-8')).digest()[:16])


# shorten name to obfuscate
def val_lic(key=None):
    if not key:
        try:
            obj = Variable.shared_qs().get(name=Variable.LICENSE_KEY)
            assert timezone.now() + timezone.timedelta(days=1) > obj.modified  # allow for clock skew
            key = obj.value
        except Exception:
            return R0

    try:
        rv = json.loads(Variable.shared_qs().get(name=Variable.LICENSE_RV).value)
        if _hash(key) in rv:
            return R0
    except Exception:
        pass

    try:
        rv = json.loads(Variable.shared_qs().get(name=Variable.LICENSE_RW).value)
        if _hash(key) in rv:
            return R0
    except Exception:
        pass

    # decode and decrypt key
    try:
        assert key.startswith('NAVWIZ_LI')
        p = base64.b64decode(key[9:])
        ciphertext, tag, nonce = struct.unpack('%ss16s12s' % (len(p) - 28), p)
        v = json.loads(AES.new(K, AES.MODE_GCM, nonce).decrypt_and_verify(ciphertext, tag))
        owner, email, features, days, till, sig = \
            v['owner'], v['email'], v['features'], v['days'], v['till'], v['sig']
        sig = base64.b64decode(sig)
    except Exception:
        return R0

    _, machine_id = myfp()
    message = '%s|%s|%s|%d|%d|%d' % (machine_id, owner, email, features, days, till)

    public_key = ECC.import_key('''-----BEGIN PUBLIC KEY-----
MCowBQYDK2VwAyEAbZjMpkVI8m01KplXGIdlvDQAR5z9DEEjoZGsnVoKSL8=
-----END PUBLIC KEY-----''')

    verifier = eddsa.new(public_key, 'rfc8032')
    try:
        verifier.verify(message.encode('utf-8'), sig)
    except ValueError:
        return R0

    if days:
        now = int(time.time())
        d = int((till - now + 86400 - 1) / 86400)
        if d <= 0 or d > days + 1:  # allow for clock skew
            return R(False, owner, email, features, -1, till)
        days = min(d, days)
    else:
        days = 0
        till = 0

    is_vm = bool(cpuid(0)[0] >= 1 and cpuid(1)[2] & (1 << 31))
    f = F_VM if is_vm else F_HW
    f |= (F_TS | F_OA if django_settings.DYNAMIC_PATH_PLANNING else F_TS) if django_settings.TRACKLESS else F_MG
    return R(features & f == f, owner, email, features, days, till)


def ret_key():
    data = dict(Variable.shared_qs().filter(
        pk__in=[Variable.LICENSE_FP, Variable.LICENSE_KEY]).values_list('name', 'value'))
    try:
        data = {
            'fingerprint': data[Variable.LICENSE_FP],
            'license_key': data[Variable.LICENSE_KEY],
        }
    except Exception:
        return

    try:
        rv = json.loads(Variable.shared_qs().get(name=Variable.LICENSE_RV).value)
        assert isinstance(rv, list)
    except Exception:
        rv = []
    rv.append(_hash(data['license_key']))

    nonce = get_random_bytes(12)
    cipher = AES.new(K, AES.MODE_GCM, nonce=nonce)
    ciphertext, tag = cipher.encrypt_and_digest(json.dumps(data).encode('utf-8'))
    key = 'NAVWIZ_RV' + base64.b64encode(b'%s%s%s' % (ciphertext, tag, nonce)).decode()

    Variable.shared_qs().update_or_create(pk=Variable.LICENSE_RET, defaults={'value': key})
    Variable.shared_qs().update_or_create(pk=Variable.LICENSE_RV, defaults={'value': json.dumps(rv)})
    Variable.shared_qs().filter(pk=Variable.LICENSE_KEY).delete()
    return key


def val_totp(token):
    return _totp('_mbjhb0bCn+Th4Y#)H$T]AvqDBE$rN*#&opw#9l][zOUMz@JIF')[0].verify(token, None, 1)


def val_totp2(token):
    return _totp('}U_XQdm1-:sRH^}>(GbZ)vW9Sz=1%kImE6hZzRT7ZI0<#9s<xa')[0].verify(token, None, 1)


def gen_totp():
    t, m = _totp('TaX055uQZ(Fe24gvXcMCeAQ9EzIPj]_Yr)j1UI8uo_x-B#U8)g')
    return m, t.now()


def _totp(secret):
    try:
        ev = Variable.shared_qs().get(name=Variable.LICENSE_FP).value
        machine_id = _hash(ev)
    except Exception:
        _, machine_id = myfp()
    s = base64.b32encode((secret + machine_id).encode('utf-8'))
    return pyotp.TOTP(s, 9, interval=12 * 3600), machine_id


# Map attribute enums

class Direction:
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    NA = 4
    MAX_N = 5


class PathFlow:
    BI_DIRECTIONAL = 0
    UNI_DIRECTIONAL = 1
    MAX_N = 2


class PathFacing:
    FREE = 0
    FORWARD_UNI = 1
    REVERSE_UNI = 2
    FORWARD_BI = 3
    REVERSE_BI = 4
    MAX_N = 5


class PathShape:
    STRAIGHT = 0
    S_CURVE = 1
    BEND_LEFT = 2
    BEND_RIGHT = 3
    TELEPORT = 4
    MAX_N = 5


class PathBranch:
    RIGHT = 0
    LEFT = 1
    NONE = 2
    MAX_N = 3


# Status code enum

class ErrorCode:
    NORMAL = 0
    PAUSED = 1

    BUMPER_BLOCKED = 100
    EXTERNAL_SAFETY_TRIGGER = 101
    EMERGENCY_BUTTON_PRESSED = 102
    CHARGER_CONNECTED = 103
    LASER_MALFUNCTION = 104
    LINE_SENSOR_ERROR = 105
    DRIVE_OVERLIMIT_ERROR = 106
    MOTOR_FAULT = 107
    WHEEL_SLIPPAGE = 108

    NAVIGATION_FAILED = 161
    OUT_OF_LINE = 162
    OBSTACLE_BLOCKED = 163
    PATH_BLOCKED = 164
    SYSTEM_ERROR = 191
    SAFETY_TRIGGERED = 199

    WAITING_TRAFFIC = 200
    WAITING_USER = 201

    DFLEET_DISCONNECTED = 300
    DFLEET_INCOMPATIBLE = 301
    DFLEET_SYNCING = 302
    DFLEET_SYNC_PENDING = 303


class Activity(ErrorCode):
    IDLE = 0
    PAUSED = 1
    WORKING = 2
    CHARGING = 3
    RUNNING_APP = 10
    NOT_READY = 80
    POWERED_OFF = 81
    UNPLANNED_MAINTENANCE = 82
    PLANNED_MAINTENANCE = 83
    SYSTEM_FAILURE = 84
    HARDWARE_FAILURE = 85

    @classmethod
    def choices(cls):
        if not getattr(cls, '_choices', None):
            # get all members of the class
            members = inspect.getmembers(cls, lambda m: not inspect.isroutine(m))
            # filter down to just properties
            props = [m for m in members if m[0][:2] != '__' and m[0] != 'NORMAL']
            # format into django choice tuple
            cls._choices = tuple(sorted([(p[1], p[0].replace('_', ' ').title()) for p in props]))
        return cls._choices

    @classmethod
    def title(cls, index):
        for idx, name in cls.choices():
            if idx == index:
                return name


@python_2_unicode_compatible
class AgvActivity(models.Model):
    start = models.DateTimeField()
    end = models.DateTimeField(unique=True)
    activity = models.PositiveSmallIntegerField(choices=Activity.choices())

    class Meta:
        verbose_name_plural = 'agv activities'

    def __str__(self):
        return '%s - %s' % (
            timezone.localtime(self.start).strftime('%Y-%m-%d %H:%M:%S'),
            timezone.localtime(self.end).strftime('%Y-%m-%d %H:%M:%S'))

    def get_duration_display(self):
        d = self.end - self.start
        h = d.seconds // 3600
        m = (d.seconds % 3600) // 60
        s = d.seconds % 60

        if d.days:
            return '%dd, %dh %dm %ds' % (d.days, h, m, s)
        elif h:
            return '%dh %dm %ds' % (h, m, s)
        elif m:
            return '%dm %ds' % (m, s)
        else:
            return '%ds' % s


@python_2_unicode_compatible
class AgvStatistic(models.Model):
    date = models.DateField(primary_key=True)
    by_activity = models.TextField()

    class Meta:
        ordering = ('date', )

    def __str__(self):
        return '%s' % self.date


@python_2_unicode_compatible
class Map(models.Model):
    name = models.CharField(max_length=127, unique=True, blank=True, null=True, default=None)
    is_named = models.BooleanField(default=False)
    created = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ('-is_named', 'name', '-created')

    def __str__(self):
        return self.name if self.is_named else (
            '~Untitled~ [%s]' % timezone.localtime(self.created).strftime('%Y-%m-%d %H:%M:%S'))

    def save(self, *args, **kwargs):
        if not self.name:
            self.name = None
        self.is_named = not not self.name
        super(Map, self).save(*args, **kwargs)


@python_2_unicode_compatible
class MapChangeset(models.Model):
    metadata = models.TextField(blank=True, default='')  # unused / reserved
    structure = models.TextField(blank=True, default='')
    stations = models.TextField(blank=True, default='')
    map = models.ForeignKey(Map, on_delete=models.CASCADE)
    # Todo: Add media field here.
    author = models.ForeignKey(django_settings.AUTH_USER_MODEL, on_delete=models.SET_NULL, blank=True, null=True)
    created = models.DateTimeField(auto_now_add=True, db_index=True)

    class Meta:
        ordering = ('-created', )

    def __str__(self):
        return timezone.localtime(self.created).strftime('%Y-%m-%d %H:%M:%S')

    def save(self, *args, **kwargs):
        """
        Force creating a new instance instead of updating.
        """
        self.pk = None
        super(MapChangeset, self).save(*args, **kwargs)

        """
        Remove old changesets. Keep only the most recent 100 changesets.
        """
        to_purge = self.map.mapchangeset_set.all()[100:].values_list('id', flat=True)
        self.map.mapchangeset_set.filter(pk__in=list(to_purge)).delete()


@python_2_unicode_compatible
class MapAnnotation(models.Model):
    annotations = models.TextField(blank=True, default='')
    map = models.OneToOneField(Map, on_delete=models.CASCADE)
    modified = models.DateTimeField(auto_now=True)

    class Meta:
        verbose_name = 'map annotation'

    def __str__(self):
        return '~Map Annotation~ [%s]' % self.map


@python_2_unicode_compatible
class Parameter(models.Model):
    key = models.CharField(max_length=255, primary_key=True)
    value = models.TextField(blank=True, default='', null=False)

    class Meta:
        ordering = ('key', )

        # extra permissions
        permissions = (
            ('change_protected_parameter', 'Can change protected parameter'),
        )

    def __str__(self):
        return self.key


if django_settings.TRACKLESS:
    Parameter.objects = Parameter.objects.db_manager('alt')


@python_2_unicode_compatible
class TaskTemplate(models.Model):
    name = models.CharField(max_length=127, unique=True)
    metadata = models.TextField(blank=True, default='')
    structure = models.TextField(blank=True, default='')
    category = models.CharField(max_length=127, blank=True, null=True, default=None, db_index=True)
    is_top_level = models.BooleanField('top-level template', default=True)
    is_active = models.BooleanField('active', default=True)
    created = models.DateTimeField(auto_now_add=True)
    modified = models.DateTimeField(auto_now=True)
    allowed_groups = models.ManyToManyField(Group)
    allowed_users = models.ManyToManyField(django_settings.AUTH_USER_MODEL)

    class Meta:
        ordering = ('-is_active', '-is_top_level', 'name')

        # extra permissions
        permissions = (
            ('use_tasktemplate', 'Use any task template'),
        )

    def __str__(self):
        return self.name

    @property
    def sorted_allowed_groups(self):
        return self.allowed_groups.order_by('name')

    @property
    def sorted_allowed_users(self):
        return self.allowed_users.order_by('username')


@python_2_unicode_compatible
class Task(models.Model):
    Status = IntEnum('Status',
        'Suspended Pending Cancelling In_progress Aborting ' +
        'Completed Aborted Cancelled Incomplete')

    name = models.CharField(max_length=127)
    task_template_id = models.PositiveIntegerField(null=True)
    task_template = models.CharField(max_length=127, blank=True, default='')
    params = models.TextField(blank=True, default='{}')
    fms_task_id = models.PositiveIntegerField(blank=True, null=True)
    status = models.PositiveSmallIntegerField(choices=Status.choices(), db_index=True, default=Status.Pending.value)
    progress = models.CharField(max_length=127, blank=True, default='')
    sequence = models.SmallIntegerField(default=0)
    owner = models.ForeignKey(django_settings.AUTH_USER_MODEL, on_delete=models.SET_NULL, blank=True, null=True)

    created = models.DateTimeField(auto_now_add=True)
    start_after = models.DateTimeField(null=True, blank=True)
    run_start = models.DateTimeField(null=True, blank=True)
    run_end = models.DateTimeField(null=True, blank=True, db_index=True)
    modified = models.DateTimeField(auto_now=True)

    class QuerySet(models.QuerySet):

        def running(self, ordered=True):
            q = self.filter(status__lt=Task.Status.Completed.value)
            if ordered:
                q = q.order_by('-status', 'sequence', 'id')
            return q

        def completed(self, ordered=True):
            q = self.filter(status__gte=Task.Status.Completed.value)
            if ordered:
                q = q.order_by('-run_end', '-id')
            return q

    objects = QuerySet.as_manager()

    class Meta:
        # extra permissions
        permissions = (
            ('abort_own_task', 'Can abort own task'),
            ('abort_task', 'Can abort any task'),
            ('cancel_own_task', 'Can cancel own task'),
            ('cancel_task', 'Can cancel any task'),
            ('prioritize_own_task', 'Can prioritize own task'),
            ('prioritize_task', 'Can prioritize any task'),
            ('suspend_own_task', 'Can suspend own task'),
            ('suspend_task', 'Can suspend any task'),
            ('resume_own_task', 'Can resume own task'),
            ('resume_task', 'Can resume any task'),
        )

    def __str__(self):
        return self.name

    def save(self, *args, **kwargs):
        if self.Status.In_progress.value <= self.status < self.Status.Cancelled.value:
            if not self.run_start:
                self.run_start = timezone.now()
        if self.status >= self.Status.Completed.value:
            if not self.run_end:
                self.run_end = timezone.now()
        super(Task, self).save(*args, **kwargs)

    @transaction.atomic
    def abort(self):
        t = Task.objects.select_for_update().defer('owner').filter(pk=self.pk).first()
        if t and t.status == self.Status.In_progress.value:
            self.status = self.Status.Aborting.value
            t.status = self.status
            t.save()

    @transaction.atomic
    def cancel(self):
        t = Task.objects.select_for_update().defer('owner').filter(pk=self.pk).first()
        if t and t.status < self.Status.In_progress.value:
            self.status = self.Status.Cancelled.value
            t.status = self.status
            t.save()

    def prioritize(self):
        self.sequence = Task.objects.running(ordered=False).aggregate(models.Min('sequence'))['sequence__min'] - 1
        self.save(update_fields=['sequence'])

    @transaction.atomic
    def suspend(self):
        t = Task.objects.select_for_update().defer('owner').filter(pk=self.pk).first()
        if t and t.status == self.Status.Pending.value:
            self.status = self.Status.Suspended.value
            t.status = self.status
            t.save()

    @transaction.atomic
    def resume(self, params=None):
        t = Task.objects.select_for_update().defer('owner').filter(pk=self.pk).first()
        if t and t.status == self.Status.Suspended.value:
            if params is not None:
                self.params = params
                t.params = params
            self.status = self.Status.Pending.value
            t.status = self.status
            t.save()


@python_2_unicode_compatible
class TaskStatistic(models.Model):
    date = models.DateField(primary_key=True)
    by_status = models.TextField()
    by_tt = models.TextField()

    class Meta:
        ordering = ('date', )

    def __str__(self):
        return '%s' % self.date


@python_2_unicode_compatible
class Webhook(models.Model):
    url = models.URLField(unique=True)
    verify_ssl = models.BooleanField('enable ssl verification', default=True)
    secret_token = models.CharField(max_length=64, blank=True, default='')
    events = models.TextField()
    created = models.DateTimeField(auto_now_add=True)
    modified = models.DateTimeField(auto_now=True)

    class Meta:
        ordering = ('url', )

    def __str__(self):
        return self.url

    def event_list(self):
        return self.events.strip('"').split('","')


@python_2_unicode_compatible
class Variable(models.Model):
    # Agv
    AGV_NAME = 'agv_name'
    AGV_HOME = 'agv_home'
    EXECUTOR_MODE = 'executor_mode'
    EXECUTOR_CFG = 'executor_cfg'
    FMS_METADATA = 'fms_metadata'
    # Task runner
    TASK_COUNTER = 'task_counter'
    # Multi-map, teleport, transition & map param
    ACTIVE_MAP = 'active_map'
    TELEPORT = 'teleport'
    TRANSITION_TRIGGER = 'transition_trigger'
    MAP_PARAM = 'map_param'
    # Global param
    GLOBAL_PARAM = 'global_param'
    # Variable
    VARIABLE = 'variable'
    # Agv Activity Config
    AGV_ACTIVITY_CONFIG = 'agv_activity_config'
    # assembly info
    AGV_MODEL = 'agv_model'
    SERIAL_NUMBER = 'serial_number'
    MANUFACTURE_DATE = 'manufacture_date'
    AGV_PARTS = 'agv_parts'
    # preventive maintenance
    NEXT_PM_DUE = 'next_pm_due'
    NEXT_PM_MILEAGE = 'next_pm_mileage'
    MILEAGE = '/agv05_motor/mileage'
    # service logs
    SERVICE_LOGS = 'service_logs'
    # license info
    LICENSE_FP = 'license_fp'
    LICENSE_KEY = 'license_key'
    LICENSE_RET = 'license_ret'
    LICENSE_RV = 'license_rv'
    LICENSE_RW = 'license_rw'
    # white label info
    ADDRESS_LABEL = 'address_label'
    COPYRIGHT_LABEL = 'copyright_label'
    FAVICON_LABEL = 'favicon_label'
    LOGO_LABEL = 'logo_label'
    # IO name
    IO_NAME = 'io_name'

    # protected variables
    PROTECTED = [AGV_MODEL, SERIAL_NUMBER, MANUFACTURE_DATE, AGV_PARTS,
        NEXT_PM_DUE, NEXT_PM_MILEAGE, MILEAGE, SERVICE_LOGS]

    name = models.CharField(max_length=127, primary_key=True)
    value = models.TextField(blank=True, default='')
    created = models.DateTimeField(auto_now_add=True)
    modified = models.DateTimeField(auto_now=True)

    class Meta:
        # extra permissions
        permissions = (
            # misc settings
            ('backup_restore', 'Can backup and restore settings'),
            ('change_agv', 'Can change agv'),
            ('change_agv_activities_config', 'Can change agv activities configuration'),
            ('change_datetime', 'Can change date and time settings'),
            ('change_license', 'Can change license key'),
            ('change_network', 'Can change network configuration'),
            ('change_own_password', 'Can change own password'),
            ('update_software', 'Can update software'),
            # transaction
            ('abort_transaction', 'Can abort any transaction'),
            ('cancel_transaction', 'Can cancel any transaction'),
            ('resume_transaction', 'Can resume any transaction'),
            # ui sections
            ('view_agv_activities', 'Can view agv activities'),
            ('view_completed_tasks', 'Can view completed tasks'),
            ('view_help_content', 'Can view help content'),
            ('view_log_files', 'Can view log files'),
            ('view_map_quality', 'Can view map quality'),
            ('view_panel', 'Can view UserPanel'),
            ('view_system_panel', 'Can view ConfigPanel'),
            ('view_users', 'Can view users'),
        )

    def __str__(self):
        return self.name

    @classmethod
    def shared_atomic(cls):
        return transaction.atomic(using='alt' if django_settings.TRACKLESS else None)

    @classmethod
    def shared_qs(cls):
        qs = cls.objects
        if django_settings.TRACKLESS:
            qs = qs.using('alt')
        return qs

    @classmethod
    def get_agv_uuid(cls):
        return myfp()[1]

    @classmethod
    def get_mileage(cls):
        try:
            return float(cls.shared_qs().get(pk=cls.MILEAGE).value)
        except Exception:
            return 0

    @classmethod
    def set_mileage(cls, mileage):
        cls.shared_qs().update_or_create(pk=cls.MILEAGE, defaults={'value': mileage})

    @classmethod
    def parse_date(cls, s):
        for f in ['%Y-%m-%d', '%d/%m/%Y']:  # cater for old format
            try:
                return timezone.datetime.strptime(s, f).date()
            except Exception:
                pass


# Executor Mode

ExecutorMode = IntEnum('ExecutorMode', 'Standalone DFleet')


# Monkey-patch the default group manager.

def patch_group_manager():
    __builtin_groups = ['Guest', 'Call Button', 'User', 'Supervisor']

    __old_get_queryset = Group.objects.get_queryset

    def get_queryset(self):
        # Preserve order, see http://blog.mathieu-leplatre.info/django-create-a-queryset-from-a-list-preserving-order.html
        clauses = ' '.join(['WHEN name=\'%s\' THEN %d' % (cn, i) for i, cn in enumerate(__builtin_groups)])
        clauses += ' ELSE %d' % len(__builtin_groups)
        ordering = 'CASE %s END' % clauses
        return __old_get_queryset().extra(select={'ordering': ordering}, order_by=('ordering', 'name'))

    def builtins(self):
        return self.get_queryset().filter(name__in=__builtin_groups)

    def non_builtins(self):
        return self.get_queryset().exclude(name__in=__builtin_groups)

    # Bind function to instance, see http://stackoverflow.com/a/1015405
    Group.objects.get_queryset = get_queryset.__get__(Group.objects, type(Group.objects))
    Group.objects.builtins = builtins.__get__(Group.objects, type(Group.objects))
    Group.objects.non_builtins = non_builtins.__get__(Group.objects, type(Group.objects))


patch_group_manager()


# Caching

class Cache(object):
    # Validation outcome
    Flag = IntEnum('Flag', 'Clean Dirty Invalid Valid Validating Reloadable')
    CACHE_FLAG = 'cache_flag'
    CACHE_FLAG_LOCK = 'cache_flag_lock'
    VALIDATION_DATA = 'validation_data'
    VALIDATED_MD5 = 'validated_md5'
    # Published Info
    MODELS_VERSION = 'models_version'
    MODELS_SKILLSET = 'models_skillset'
    MODELS_SKILLSET_MD5 = 'models_skillset_md5'
    MODELS_APP_DESCRIPTIONS = 'models_app_descriptions'
    MODELS_ROOTFS = 'models_rootfs'
    # Map
    MAP_PARAMS = 'map_params'
    MAP_STRUCTURE = 'map_structure'
    STATION_NAMES = 'station_names'
    # Global parameter
    GLOBAL_PARAMS = 'global_params'
    # Variable
    VARIABLES = 'variables'
    # Task template
    TASK_TEMPLATES = 'task_templates'
    # Teleport
    TELEPORTS = 'teleports'
    # Transition Triggers
    TRANSITION_TRIGGERS = 'transition_triggers'
    # Transaction
    TRANSACTION_ENABLED = 'transaction_enabled'
    # Agv
    AGV_UUID = 'agv_uuid'
    AGV_NAME = 'agv_name'
    AGV_HOME = 'agv_home'
    EXECUTOR_MODE = 'executor_mode'
    EXECUTOR_CFG = 'executor_cfg'
    FMS_METADATA = 'fms_metadata'
    # Aggregation
    FMS_SKILLSET = 'fms_skillset'
    DOWNLOADABLES = 'downloadables'
    DOWNLOADABLES_MD5 = 'downloadables_md5'
    # Dashboard
    DASHBOARD_STATS = 'dashboard_stats'

    @classmethod
    def set_flag(cls, flag):
        assert isinstance(flag, int)
        cache.set(cls.CACHE_FLAG, flag, None)

    @classmethod
    def get_flag(cls):
        return cache.get_or_set(cls.CACHE_FLAG, cls.Flag.Dirty.value)

    @classmethod
    def set_flag_if(cls, flag, if_flags):
        if not isinstance(if_flags, (list, tuple)):
            if_flags = (if_flags, )

        assert isinstance(flag, int)
        assert all(isinstance(if_flag, int) for if_flag in if_flags)

        while not redis_cache.add('cache_flag_lock', True, 5):
            time.sleep(0.1)

        if cls.get_flag() in if_flags:
            cls.set_flag(flag)

        redis_cache.delete('cache_flag_lock')

    @classmethod
    def set_validation_data(cls, data):
        cache.set(cls.VALIDATION_DATA, data, None)

    @classmethod
    def get_validation_data(cls):
        return cache.get(cls.VALIDATION_DATA)

    @classmethod
    def get_validation_msg(cls):
        data = cls.get_validation_data()
        if data:
            return data['error_msg'].format(**{k: v['name'] for k, v in data['params'].items()})
        return ''

    @classmethod
    def set_validated_md5(cls, validated_md5):
        cache.set(cls.VALIDATED_MD5, validated_md5, None)

    @classmethod
    def get_validated_md5(cls):
        return cache.get(cls.VALIDATED_MD5)

    @classmethod
    def set_models_version(cls, models_version):
        cache.set(cls.MODELS_VERSION, models_version, None)

    @classmethod
    def get_models_version(cls):
        return cache.get(cls.MODELS_VERSION)

    @classmethod
    def set_models_skillset(cls, models_skillset):
        cache.set(cls.MODELS_SKILLSET, models_skillset, None)

    @classmethod
    def get_models_skillset(cls):
        return cache.get(cls.MODELS_SKILLSET)

    @classmethod
    def set_models_skillset_md5(cls, models_skillset_md5):
        cache.set(cls.MODELS_SKILLSET_MD5, models_skillset_md5, None)

    @classmethod
    def get_models_skillset_md5(cls):
        return cache.get(cls.MODELS_SKILLSET_MD5)

    @classmethod
    def set_models_app_descriptions(cls, models_app_descriptions):
        cache.set(cls.MODELS_APP_DESCRIPTIONS, models_app_descriptions, None)

    @classmethod
    def get_models_app_descriptions(cls):
        return cache.get(cls.MODELS_APP_DESCRIPTIONS)

    @classmethod
    def set_models_rootfs(cls, models_app_descriptions):
        cache.set(cls.MODELS_ROOTFS, models_app_descriptions, None)

    @classmethod
    def get_models_rootfs(cls):
        return cache.get(cls.MODELS_ROOTFS)

    @classmethod
    def set_map_params(cls, map_params):
        cache.set(cls.MAP_PARAMS, map_params, None)

    @classmethod
    def get_map_params(cls):
        return cache.get(cls.MAP_PARAMS)

    @classmethod
    def set_map_structure(cls, map_structure):
        cache.set(cls.MAP_STRUCTURE, map_structure, None)

    @classmethod
    def get_map_structure(cls):
        return cache.get(cls.MAP_STRUCTURE)

    @classmethod
    def set_station_names(cls, station_names):
        cache.set(cls.STATION_NAMES, station_names, None)

    @classmethod
    def get_station_names(cls):
        return cache.get(cls.STATION_NAMES)

    @classmethod
    def set_global_params(cls, global_params):
        cache.set(cls.GLOBAL_PARAMS, global_params, None)

    @classmethod
    def get_global_params(cls):
        return cache.get(cls.GLOBAL_PARAMS)

    @classmethod
    def set_variables(cls, variables):
        cache.set(cls.VARIABLES, variables, None)

    @classmethod
    def get_variables(cls):
        return cache.get(cls.VARIABLES, [])

    @classmethod
    def set_task_templates(cls, task_templates):
        cache.set(cls.TASK_TEMPLATES, task_templates, None)

    @classmethod
    def get_task_templates(cls):
        return cache.get(cls.TASK_TEMPLATES, [])

    @classmethod
    def get_task_template(cls, template_id=None, template_name=None):
        task_templates = cls.get_task_templates()

        if template_id:
            for tt in task_templates:
                if tt['id'] == template_id:
                    return tt

        if template_name:
            for tt in task_templates:
                if tt['name'] == template_name:
                    return tt

    @classmethod
    def get_task_template_if_allowed(cls, user, template_id=None, template_name=None, top_level_only=True):
        if user.is_anonymous:
            user = get_user_model().objects.filter(username='Guest').first()

        if not user:
            return

        group = user.groups.first()

        tt = cls.get_task_template(template_id, template_name)
        if not tt:
            return

        if not top_level_only or tt['is_top_level']:
            if user.has_perm('system.use_tasktemplate') or user.username in tt['allowed_users'] or (group and group.name in tt.get('allowed_groups', [])):
                return tt

        return False

    @classmethod
    def filter_task_templates(cls, user, top_level_only=True):
        if user.is_anonymous:
            user = get_user_model().objects.filter(username='Guest').first()

        if not user:
            return []

        group = user.groups.first()

        if user.has_perm('system.use_tasktemplate'):
            if top_level_only:
                return [tt for tt in cls.get_task_templates() if tt['is_top_level']]
            else:
                return cls.get_task_templates()

        return [tt for tt in cls.get_task_templates()
            if (user.username in tt['allowed_users'] or group and group.name in tt.get('allowed_groups', [])) and (not top_level_only or tt['is_top_level'])]

    @classmethod
    def set_teleports(cls, teleports):
        cache.set(cls.TELEPORTS, teleports, None)

    @classmethod
    def get_teleports(cls):
        return cache.get(cls.TELEPORTS)

    @classmethod
    def set_transition_triggers(cls, transition_triggers):
        cache.set(cls.TRANSITION_TRIGGERS, transition_triggers, None)

    @classmethod
    def get_transition_triggers(cls):
        return cache.get(cls.TRANSITION_TRIGGERS)

    @classmethod
    def set_transaction_enabled(cls, state):
        cache.set(cls.TRANSACTION_ENABLED, state, None)

    @classmethod
    def get_transaction_enabled(cls):
        return cache.get(cls.TRANSACTION_ENABLED)

    @classmethod
    def set_agv_uuid(cls, agv_uuid):
        cache.set(cls.AGV_UUID, agv_uuid, None)

    @classmethod
    def get_agv_uuid(cls):
        return cache.get(cls.AGV_UUID)

    @classmethod
    def set_agv_name(cls, agv_name):
        cache.set(cls.AGV_NAME, agv_name, None)

    @classmethod
    def get_agv_name(cls):
        return cache.get_or_set(cls.AGV_NAME, 'AGV05')

    @classmethod
    def set_agv_home(cls, agv_home):
        cache.set(cls.AGV_HOME, agv_home, None)

    @classmethod
    def get_agv_home(cls):
        return cache.get(cls.AGV_HOME)

    @classmethod
    def set_executor_mode(cls, executor_mode):
        cache.set(cls.EXECUTOR_MODE, executor_mode, None)

    @classmethod
    def get_executor_mode(cls):
        return cache.get_or_set(cls.EXECUTOR_MODE, ExecutorMode.Standalone.value)

    @classmethod
    def set_executor_cfg(cls, executor_cfg):
        cache.set(cls.EXECUTOR_CFG, executor_cfg, None)

    @classmethod
    def get_executor_cfg(cls):
        return cache.get(cls.EXECUTOR_CFG)

    @classmethod
    def set_fms_metadata(cls, fms_metadata):
        cache.set(cls.FMS_METADATA, fms_metadata, None)

    @classmethod
    def get_fms_metadata(cls):
        return cache.get(cls.FMS_METADATA)

    @classmethod
    def set_fms_skillset(cls, fms_skillset):
        cache.set(cls.FMS_SKILLSET, fms_skillset, None)

    @classmethod
    def get_fms_skillset(cls):
        return cache.get(cls.FMS_SKILLSET)

    @classmethod
    def set_downloadables(cls, downloadables):
        cache.set(cls.DOWNLOADABLES, downloadables, None)

    @classmethod
    def get_downloadables(cls):
        return cache.get(cls.DOWNLOADABLES)

    @classmethod
    def set_downloadables_md5(cls, downloadables_md5):
        cache.set(cls.DOWNLOADABLES_MD5, downloadables_md5, None)

    @classmethod
    def get_downloadables_md5(cls):
        return cache.get(cls.DOWNLOADABLES_MD5)

    @classmethod
    def set_dashboard_stats(cls, dashboard_stats):
        cache.set(cls.DASHBOARD_STATS, dashboard_stats, None)

    @classmethod
    def get_dashboard_stats(cls):
        return cache.get(cls.DASHBOARD_STATS, {})
