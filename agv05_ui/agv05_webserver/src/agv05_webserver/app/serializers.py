from __future__ import absolute_import
from __future__ import unicode_literals

from collections import OrderedDict
from django.contrib.auth import get_user_model
from django.contrib.auth.hashers import make_password
from django.contrib.auth.models import Group, Permission
from django.core.files.storage import default_storage
from django.utils import timezone
from enumfields import IntEnum
from enumfields.drf import EnumField
from itertools import chain
from rest_framework import serializers
from rest_framework.reverse import reverse
import math
import rospy
import std_msgs.msg
import std_srvs.srv
import string
import ujson as json

from agv05_webserver.system.models import (
    AgvActivity, AgvStatistic, Cache, Map, MapAnnotation, MapChangeset,
    Parameter, Task, TaskStatistic, TaskTemplate,
    Variable, Webhook,
)

User = get_user_model()


class VariableSerializer(serializers.ModelSerializer):
    overwrite = serializers.BooleanField(required=False)
    modified = serializers.CharField(required=False)

    class Meta:
        model = Variable
        fields = ('value', 'modified', 'overwrite')


class AgvSerializer(serializers.Serializer):
    uuid = serializers.UUIDField()
    name = serializers.CharField()
    status = serializers.CharField()
    action = serializers.CharField()
    location_hint = serializers.CharField(required=False, default='')
    location = serializers.JSONField()
    prev_location = serializers.JSONField()
    pose = serializers.JSONField()
    motion = serializers.CharField()
    velocity = serializers.JSONField()
    error_code = serializers.IntegerField()
    safety_message = serializers.JSONField()
    user_message = serializers.JSONField()
    charging = serializers.BooleanField()
    battery = serializers.FloatField()
    task_counter = serializers.IntegerField()
    mileage = serializers.FloatField()
    fms_status = serializers.CharField()


class AgvActivityViewSerializer(serializers.ModelSerializer):
    activity_duration = serializers.SerializerMethodField('get_duration_display')
    activity_display = serializers.SerializerMethodField('get_display')

    def get_duration_display(self, item):
        return item.get_duration_display()

    def get_display(self, item):
        return item.get_activity_display()

    class Meta:
        model = AgvActivity
        fields = ['id', 'start', 'end', 'activity_duration', 'activity_display', 'activity']
        read_only_fields = ('id', 'start', 'end', 'activity_duration', 'activity_display', 'activity')


class AgvActivityChoicesSerializer(serializers.Serializer):
    pass


class AgvActivityEditSerializer(serializers.ModelSerializer):
    class Meta:
        model = AgvActivity
        fields = ['start', 'end', 'duration', 'activity']
        read_only_fields = ['start', 'end', 'duration']


class AgvActivityConfigSerializer(VariableSerializer):
    activities = serializers.CharField(read_only=True)

    class Meta(VariableSerializer.Meta):
        fields = ('value', 'activities', 'modified', 'overwrite')


class AgvActivityArchiveSerializer(serializers.Serializer):
    month = serializers.DateField()
    month_number = serializers.IntegerField()
    exists = serializers.BooleanField()


class AgvStatisticSerializer(serializers.ModelSerializer):
    class Meta:
        model = AgvStatistic
        fields = '__all__'


class AppSerializer(serializers.Serializer):

    class Operation(IntEnum):
        START_APP = 1
        STOP_APP = 0

    operation = EnumField(enum=Operation, lenient=True, ints_as_names=True)
    app_id = serializers.CharField()


class PartSerializer(serializers.Serializer):
    module = serializers.CharField(allow_blank=True)
    part = serializers.CharField(allow_blank=True)
    serial = serializers.CharField(allow_blank=True)
    remarks = serializers.CharField(allow_blank=True)
    maintenance = serializers.JSONField()
    status = serializers.CharField(allow_blank=True)


class AssemblyInfoSerializer(serializers.Serializer):
    agv_model = serializers.CharField()
    serial_number = serializers.CharField()
    manufacture_date = serializers.DateField()
    overwrite_mileage = serializers.BooleanField(required=False, write_only=True)
    mileage = serializers.IntegerField(required=False, min_value=0)
    parts = PartSerializer(many=True, required=False)
    maker_key = serializers.CharField(max_length=9, write_only=True)
    maker_key_2 = serializers.CharField(max_length=9, write_only=True)

    def to_internal_value(self, data):
        # alias for fields from DF Hub
        if 'model' in data and 'agv_model' not in data:
            data['agv_model'] = data.pop('model')
        if 'serial' in data and 'serial_number' not in data:
            data['serial_number'] = data.pop('serial')
        return super(AssemblyInfoSerializer, self).to_internal_value(data)

    def validate(self, data):
        if data.get('overwrite_mileage') and data.get('mileage') is None:
            raise serializers.ValidationError({'mileage': 'This field is required.'})
        return data


class PreventiveMaintenanceSerializer(serializers.Serializer):
    next_pm_due = serializers.DateField()
    next_pm_mileage = serializers.IntegerField(min_value=0)
    mileage = serializers.IntegerField(min_value=0, read_only=True)
    maker_key = serializers.CharField(max_length=9, write_only=True)


class ServiceLogSerializer(serializers.Serializer):
    report_no = serializers.CharField(allow_blank=True)
    title = serializers.CharField()
    vars()['type'] = serializers.CharField()
    start_time = serializers.DateTimeField()
    end_time = serializers.DateTimeField()
    mileage = serializers.IntegerField(required=False)
    progress_comment = serializers.CharField(allow_blank=True)
    next_milestone = serializers.CharField(allow_blank=True)


class ServiceLogsSerializer(serializers.Serializer):
    service_logs = ServiceLogSerializer(many=True)
    maker_key = serializers.CharField(max_length=9, write_only=True)


class BootSerializer(serializers.Serializer):

    class Operation(IntEnum):
        EXTEND_COUNTDOWN = 10
        START_ROBOT = 1
        START_ROBOT_2 = 2
        START_ROBOT_3 = 3
        STOP_ROBOT = 0
        SOFT_REBOOT = -1
        HARD_REBOOT = -2
        POWER_OFF = -3
        SAFE_SOFT_REBOOT = -4
        HOT_RELOAD = -5

    operation = EnumField(enum=Operation, lenient=True, ints_as_names=True)


class FmsTaskSerializer(serializers.Serializer):
    fms_task_id = serializers.IntegerField()


class _HardwarePluginField(serializers.Serializer):
    name = serializers.CharField(required=False)
    upload = serializers.FileField(required=False)


class HardwarePluginSerializer(serializers.Serializer):
    plugins = serializers.ListField(child=_HardwarePluginField(), max_length=5)


class LicenseInfoSerializer(serializers.Serializer):
    valid = serializers.BooleanField()
    owner = serializers.CharField()
    email = serializers.CharField()
    features = serializers.IntegerField()
    days = serializers.IntegerField()
    till = serializers.IntegerField()


class LicenseOfflineSerializer(serializers.Serializer):
    license_key = serializers.CharField()


class LicenseOnlineSerializer(serializers.Serializer):
    activation_key = serializers.CharField(max_length=40)


class LicenseRequestSerializer(serializers.Serializer):
    machine_id = serializers.CharField()
    license_req = serializers.CharField()
    license_ret = serializers.CharField()


class MapSerializer(serializers.ModelSerializer):
    structure = serializers.SerializerMethodField()

    class Meta:
        model = Map
        fields = ('id', 'name', 'structure')
        read_only_fields = ('id', 'name')

    def get_structure(self, obj):
        struc = ''
        try:
            struc = MapChangeset.objects.filter(map=obj).first().structure
        except Exception:
            pass
        return struc


class MapListSerializer(serializers.ModelSerializer):
    active = serializers.SerializerMethodField()

    class Meta:
        model = Map
        fields = ('id', 'name', 'active')
        read_only_fields = ('id', 'name')

    def __init__(self, *args, **kwargs):
        try:
            self.active_map_id = Variable.objects.get(name=Variable.ACTIVE_MAP).value.split(',')
        except Variable.DoesNotExist:
            self.active_map_id = []
        return super(MapListSerializer, self).__init__(*args, **kwargs)

    def get_active(self, obj):
        return str(obj.pk) in self.active_map_id

    def to_representation(self, instance):
        data = super(MapListSerializer, self).to_representation(instance)
        data['name'] = str(instance)
        return data


class MapEditSerializer(serializers.ModelSerializer):
    display_name = serializers.SerializerMethodField()

    class Meta:
        model = Map
        fields = ('id', 'name', 'display_name')
        read_only_fields = ('id', )

    def get_display_name(self, obj):
        return str(obj)


class MapReservedParamMixin(object):

    def get_reserved_params(self, obj):
        return json.dumps([{
            'name': 'Unlimited',
            'type': 'double',
        }])


class MapLayoutMixin(MapReservedParamMixin):

    def create(self, validated_data, *args, **kwargs):
        if 'overwrite' in validated_data:
            del validated_data['overwrite']
        return super(MapLayoutMixin, self).create(validated_data, *args, **kwargs)

    def get_params(self, obj):
        params = '[]'
        try:
            raw = Variable.objects.get(pk=Variable.MAP_PARAM).value
            if raw:
                params = raw
        except Variable.DoesNotExist:
            pass

        return params


class MapLayoutSerializer(MapLayoutMixin, serializers.ModelSerializer):
    overwrite = serializers.BooleanField(required=False)
    modified = serializers.CharField(required=False, source='created')
    params = serializers.SerializerMethodField()
    reserved_params = serializers.SerializerMethodField()

    class Meta:
        model = MapChangeset
        fields = ('metadata', 'structure', 'stations', 'reserved_params',
                  'params', 'modified', 'overwrite')


class MapAnnotationSerializer(serializers.ModelSerializer):
    overwrite = serializers.BooleanField(required=False)
    modified = serializers.CharField(required=False)

    def create(self, validated_data, *args, **kwargs):
        del validated_data['overwrite']
        return super(MapAnnotationSerializer, self).create(validated_data, *args, **kwargs)

    class Meta:
        model = MapAnnotation
        fields = ('annotations', 'modified', 'overwrite')


class MapChangesetSummarySerializer(serializers.ModelSerializer):
    author = serializers.StringRelatedField()

    class Meta:
        model = MapChangeset
        fields = ('id', 'author', 'created')


class MapChangesetSerializer(serializers.ModelSerializer):
    author = serializers.StringRelatedField()

    class Meta:
        model = MapChangeset
        fields = ('id', 'metadata', 'structure', 'stations', 'author', 'created')


class MapActiveSerializer(VariableSerializer):
    results = serializers.SerializerMethodField()

    class Meta:
        model = Variable
        fields = ('value', 'modified', 'overwrite', 'results')

    def get_results(self, obj):
        try:
            a = Variable.objects.get(name=Variable.ACTIVE_MAP)
            value = a.value
            modified = a.modified
        except Exception:
            value = ''
            modified = timezone.now() - timezone.timedelta(days=1)

        try:
            query = Map.objects.filter(pk__in=value.split(','))
        except Exception:
            query = None

        serializer = MapListSerializer(query, many=True)
        return serializer.data


class MapParamSerializer(MapReservedParamMixin, VariableSerializer):
    reserved_params = serializers.SerializerMethodField()

    class Meta:
        model = Variable
        fields = ('value', 'modified', 'overwrite', 'reserved_params')


class RegisterSerializer(serializers.Serializer):
    name = serializers.CharField(required=False)
    value = serializers.IntegerField()

    class Meta:
        read_only_fields = ('name', )


class TaskTemplateVariableSerializer(serializers.Serializer):

    name = serializers.CharField(required=False)
    type = serializers.CharField(required=False)
    value = serializers.JSONField()

    class Meta:
        read_only_fields = ('name', 'type')


class SoftwarePatchSerializer(serializers.Serializer):
    patch_file = serializers.FileField()


class SoftwareUpdateUsbInfoSerializer(serializers.Serializer):
    usb = serializers.ChoiceField([])


class SoftwareUpdateUsbSerializer(serializers.Serializer):
    usb = serializers.ChoiceField([])
    version = serializers.CharField()
    install_kiosk = serializers.BooleanField(required=False)


class TaskSerializer(serializers.ModelSerializer):
    name = serializers.CharField(default='', allow_blank=True)
    params = serializers.JSONField(default={})
    status = EnumField(enum=Task.Status, lenient=True,
                       ints_as_names=True, required=False, read_only=True)
    owner = serializers.StringRelatedField()

    class Meta:
        model = Task
        fields = ('id', 'name', 'task_template_id', 'task_template', 'params',
                  'fms_task_id', 'status', 'progress', 'sequence', 'owner',
                  'created', 'run_start', 'run_end')
        read_only_fields = ('fms_task_id', 'status', 'owner',
                            'created', 'run_start', 'run_end')


class TaskCompletedSerializer(TaskSerializer):
    duration = serializers.SerializerMethodField()
    executed = serializers.SerializerMethodField()

    def get_executed(self, item):
        if item.status < Task.Status.Cancelled.value:
            return True
        return False

    def get_duration(self, item):
        if item.run_start and item.run_end:
            d = item.run_end - item.run_start
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
        else:
            return '-'

    class Meta:
        model = Task
        fields = ('id', 'name', 'task_template_id', 'task_template', 'params',
                  'fms_task_id', 'status', 'progress', 'sequence', 'owner',
                  'created', 'run_start', 'run_end', 'duration', 'executed', 'start_after')
        read_only_fields = ('fms_task_id', 'status', 'owner',
                            'created', 'run_start', 'run_end', 'duration', 'executed', 'start_after')


class TaskCompletedArchiveSerializer(serializers.Serializer):
    month = serializers.DateField()
    month_number = serializers.IntegerField()
    exists = serializers.BooleanField()


class TaskStatisticSerializer(serializers.ModelSerializer):
    class Meta:
        model = TaskStatistic
        fields = '__all__'


class TaskTemplateSerializer(serializers.ModelSerializer):
    allowed_groups = serializers.StringRelatedField(many=True)
    allowed_users = serializers.StringRelatedField(many=True)

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'metadata', 'structure', 'category', 'is_top_level', 'is_active',
                  'created', 'modified', 'allowed_groups', 'allowed_users')


class TaskTemplateListSerializer(serializers.ModelSerializer):
    allowed_groups = serializers.StringRelatedField(many=True)
    allowed_users = serializers.StringRelatedField(many=True)

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'category', 'is_top_level', 'is_active',
                  'allowed_groups', 'allowed_users')


class TaskTemplateEditSerializer(serializers.ModelSerializer):
    overwrite = serializers.BooleanField(required=False)
    modified = serializers.CharField(required=False)
    cached = serializers.SerializerMethodField()

    def get_cached(self, obj):
        return Cache.get_task_template(template_id=obj.pk) is not None

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'metadata', 'structure', 'category', 'is_top_level', 'is_active',
                  'created', 'modified', 'overwrite', 'cached')


class TaskTemplateUserSerializer(serializers.ModelSerializer):
    allowed_users = serializers.PrimaryKeyRelatedField(
        queryset=User.objects.filter(is_staff=False).order_by('username'),
        many=True
    )
    allowed_groups = serializers.PrimaryKeyRelatedField(
        queryset=Group.objects.all(),
        many=True
    )

    available_users = serializers.SerializerMethodField()
    available_groups = serializers.SerializerMethodField()

    def get_available_users(self, obj):
        return User.objects \
            .filter(is_staff=False) \
            .order_by('username') \
            .values_list('id', 'username')

    def get_available_groups(self, obj):
        return Group.objects.all().values_list('id', 'name')

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'allowed_users', 'allowed_groups', 'available_users', 'available_groups')
        read_only_fields = ('id', 'name', 'available_users', 'available_groups')


class TaskTemplateMassUpdateSerializer(serializers.ModelSerializer):
    class CreateSuspendedField(serializers.BooleanField):
        def to_representation(self, obj):
            try:
                metadata = json.loads(obj)
                return metadata.get('create_suspended', False)
            except Exception:
                pass
            return True

    is_active = serializers.BooleanField(required=False)
    is_top_level = serializers.BooleanField(required=False)
    create_suspended = CreateSuspendedField(source='metadata', required=False)

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'is_top_level', 'is_active', 'create_suspended')
        read_only_fields = ('id', 'name')


class TaskTemplateMassDeleteSerializer(serializers.ModelSerializer):

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name')
        read_only_fields = ('id', 'name')


class TaskTemplateMassUserSerializer(serializers.ModelSerializer):
    allowed_users = serializers.PrimaryKeyRelatedField(
        queryset=User.objects.filter(is_staff=False).order_by('username'),
        many=True
    )
    allowed_groups = serializers.PrimaryKeyRelatedField(
        queryset=Group.objects.all(),
        many=True
    )

    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'allowed_users', 'allowed_groups')
        read_only_fields = ('id', 'name')


class TaskTemplateMassCategoryUpdateSerializer(serializers.ModelSerializer):
    class Meta:
        model = TaskTemplate
        fields = ('id', 'name', 'category')
        read_only_fields = ('id', 'name')


class TransactionSerializer(serializers.Serializer):
    pass


class TransactionResumeSerializer(serializers.Serializer):
    action = serializers.CharField()
    error = serializers.IntegerField()


class WebhookSerializer(serializers.ModelSerializer):
    class Meta:
        model = Webhook
        fields = ('id', 'url', 'verify_ssl', 'events')


class WebhookEditSerializer(serializers.ModelSerializer):
    class Meta:
        model = Webhook
        fields = '__all__'


class DateTimeSerializer(serializers.Serializer):
    TIMEZONE_CHOICES = [(x, x) for x in timezone.pytz.common_timezones]

    date_and_time = serializers.DateTimeField()
    time_zone = serializers.ChoiceField(TIMEZONE_CHOICES)
    ntp_sync = serializers.BooleanField(
        label='Auto synchronize with network time server',
    )
    ntp_server = serializers.IPAddressField(
        label='Local NTP Server', protocol='ipv4', required=False, allow_blank=True,
        help_text='The IP address of a preferred network time server for synchronization (optional).'
    )
    ntp_status = serializers.CharField(read_only=True)
    time_zone_choices = serializers.SerializerMethodField()

    def get_time_zone_choices(self, d):
        return self.TIMEZONE_CHOICES


class LogRequestSerializer(serializers.Serializer):
    start_date = serializers.DateField()
    end_date = serializers.DateField()

    def validate(self, data):
        if data['start_date'] > data['end_date']:
            raise serializers.ValidationError({'start_date': 'Start date must be before end date.'})
        if data['end_date'] - data['start_date'] > timezone.timedelta(days=5):
            raise serializers.ValidationError({'start_date': 'Duration cannot be more then 5 days.'})
        return data


class LogDownloadSerializer(serializers.Serializer):
    request_id = serializers.UUIDField()


class WifiOpSerializer(serializers.Serializer):
    ADHOC = 0
    WIFILIST = 1
    WNICS = 2
    STATUS = 3
    WIFIOP_CHOICES = (
        (ADHOC, 'Adhoc'),
        (WIFILIST, 'Wifi List'),
        (WNICS, 'Wnics'),
        (STATUS, 'Status'),
    )
    wifiop = serializers.ChoiceField(WIFIOP_CHOICES)
    state = serializers.NullBooleanField(required=False)


class WifiConnectSerializer(serializers.Serializer):
    CONNECT = 0
    FORGET = 1
    WIFICON_CHOICES = (
        (CONNECT, 'Connect'),
        (FORGET, 'Forget'),
    )
    op = serializers.ChoiceField(WIFICON_CHOICES)
    ssid = serializers.CharField(allow_blank=False)
    identity = serializers.CharField(required=False, allow_blank=True)
    password = serializers.CharField(required=False, allow_blank=True, style={
                                     'input_type': 'password'})
    enc = serializers.CharField(required=False, allow_blank=False)
    eap = serializers.CharField(required=False, allow_blank=True)
    hidden = serializers.BooleanField(required=False)


class WifiIPConfigSerializer(serializers.Serializer):
    staticmode = serializers.BooleanField(required=True)
    ip = serializers.CharField(allow_blank=True, required=False)
    netmask = serializers.CharField(allow_blank=True, required=False)
    gateway = serializers.CharField(allow_blank=True, required=False)
    dns_nameserver = serializers.CharField(allow_blank=True, required=False)


class NetworkEAPSerializer(serializers.Serializer):
    download = serializers.CharField(allow_blank=True, required=False)
    ca_crt_enable = serializers.BooleanField(required=True)
    ca_crt = serializers.FileField(allow_empty_file=False, required=False)
    wifi_crt_enable = serializers.BooleanField(required=True)
    wifi_crt = serializers.FileField(allow_empty_file=False, required=False)
    wifi_crt_pass = serializers.CharField(allow_blank=True, required=False)


class LogSerializer(serializers.Serializer):
    pass


class SearchSerializer(serializers.Serializer):
    search = serializers.CharField(required=True)


class GroupSerializer(serializers.ModelSerializer):
    class Meta:
        model = Group
        fields = ('id', 'name')


class UserPassMixin(object):

    def validate(self, data):
        ret = super(UserPassMixin, self).validate(data)
        password_error = []
        if len(data['password']) < 8:
            password_error.append('Password must be at least 8 characters long')
        if data['password'].isnumeric():
            password_error.append('Password cannot be entirely numeric')

        username = self.instance.username if self.instance else data.get('username')
        if username.lower() in data['password'].lower():
            password_error.append('Password is too simlar to the username')
        if password_error:
            raise serializers.ValidationError({
                'password': password_error,
            })
        return ret


class UserGroupMixin(object):
    def validate(self, data):
        ret = super(UserGroupMixin, self).validate(data)
        if not data.get('groups') or len(data['groups']) != 1:
            raise serializers.ValidationError({
                'groups': 'User must be assigned to one group.',
            })
        return ret


class UserListSerializer(serializers.ModelSerializer):
    groups = serializers.StringRelatedField(many=True)

    class Meta:
        model = User
        fields = ('id', 'username', 'groups', 'is_active')
        read_only_fields = ('id', )


class UserSerializer(UserGroupMixin, UserPassMixin, serializers.ModelSerializer):
    groups = serializers.PrimaryKeyRelatedField(
        queryset=Group.objects.exclude(name='Guest'),
        many=True
    )

    class Meta:
        model = User
        fields = ('id', 'username', 'password', 'groups', 'is_active')
        read_only_fields = ('id', )
        extra_kwargs = {
            'password': {
                'write_only': True,
                'style': {
                    'input_type': 'password'
                }
            }
        }

    def create(self, validated_data):
        user = User.objects.create(
            username=validated_data['username'],
            is_active=validated_data['is_active'],
            password=make_password(validated_data['password']))
        user.groups.set(validated_data['groups'])
        return user


class UserUpdateSerializer(UserGroupMixin, serializers.ModelSerializer):
    groups = serializers.PrimaryKeyRelatedField(
        queryset=Group.objects.exclude(name='Guest'),
        many=True
    )

    class Meta:
        model = User
        fields = ('id', 'username', 'groups', 'is_active')
        read_only_fields = ('id', )


class UserPassUpdateSerializer(UserPassMixin, serializers.ModelSerializer):
    class Meta:
        model = User
        fields = ('id', 'username', 'password')
        read_only_fields = ('id', 'username')
        extra_kwargs = {
            'password': {
                'write_only': True,
                'style': {
                    'input_type': 'password'
                }
            }
        }

    def update(self, user, validated_data):
        user.set_password(validated_data['password'])
        user.save()
        return user


class ProtectionPinSerializer(serializers.Serializer):
    pin = serializers.RegexField(
        regex=r'^\d{6}$',
        error_messages={'invalid': 'PIN must be a 6-digit number'},
        style={
            'input_type': 'password',
        },
    )


class ParameterListSerializer(serializers.ModelSerializer):

    class Meta:
        model = Parameter
        fields = ('key', )
        read_only_fields = ('key', )


class ParameterSerializer(serializers.ModelSerializer):
    key = serializers.ReadOnlyField()

    def __init__(self, instance_data=None, *args, **kwargs):
        self.instance_data = instance_data
        super(ParameterSerializer, self).__init__(*args, **kwargs)

    def to_representation(self, instance):
        if isinstance(instance, Parameter):
            instance = self.instance_data
        return super(ParameterSerializer, self).to_representation(instance)

    class Meta:
        model = Parameter
        fields = ('key', )
        read_only_fields = ('key', )


def parameterserializer_factory(desc=None, config_msg=None, allow_protected=False):

    data_src = {
        'topic': {},
        'srv': {},
    }

    def _get_enum_src(src):
        _id = src['id']
        _type = src['type']
        if _type not in data_src:
            return
        data = data_src[_type]
        _loc = src[_type]

        if _loc in data:
            enum_dict = data[_loc]
        elif _type == 'topic':
            data[_loc] = None
            msg = rospy.wait_for_message(_loc, std_msgs.msg.String, 0.5)
            enum_dict = json.loads(msg.data)
            data[_loc] = enum_dict
        elif _type == 'srv':
            data[_loc] = None
            srv_proxy = rospy.ServiceProxy(_loc, std_srvs.srv.Trigger)
            srv_proxy.wait_for_service(0.5)
            resp = srv_proxy.call()
            if not resp.success:
                return
            enum_dict = json.loads(resp.message)
            data[_loc] = enum_dict

        return enum_dict[_id]

    def get_enum(edit_method):
        enum = []
        if 'enum_data' in edit_method:
            enum = edit_method['enum_data']
        elif 'enum_src' in edit_method:
            src = edit_method['enum_src']
            enum = _get_enum_src(src)
        return enum

    def handle_dr_app(d, edit_method, enum_choices):
        app_type = edit_method['app']
        if app_type == 'multiplechoice' and d['type'] == 'str':
            if not enum_choices:
                return

            return (
                d['name'],
                MultipleChoiceField(
                    initial=d['default'],
                    required=False,
                    help_text=d['description'],
                    choices=enum_choices,
                ),
            )

    def fields_from_param_desc(desc):
        for d in desc:
            enum_choices = []
            try:
                if d['edit_method']:
                    edit_method = eval(d['edit_method'])
                    if 'enum' in edit_method and isinstance(edit_method['enum'], list):
                        enum_choices = [(e['value'], e['name'] + (' (%s)' % e['description'] if e['description'] else '')) for e in edit_method['enum']]
                    else:
                        enum_choices = get_enum(edit_method)

                    if 'app' in edit_method:
                        dr_app = handle_dr_app(d, edit_method, enum_choices)
                        if dr_app:
                            yield dr_app
                            continue

            except Exception:
                pass

            try:
                if enum_choices:
                    yield (
                        d['name'],
                        ChoiceField(
                            choices=enum_choices,
                            initial=d['default'],
                            help_text=d['description']
                        ),
                    )
                elif d['type'] == 'bool':
                    yield (
                        d['name'],
                        BooleanField(
                            required=False,
                            initial=d['default'],
                            help_text=d['description']
                        ),
                    )

                elif d['type'] == 'int':
                    description = '%s (Range: %s &le; x &le; %s)' % (d['description'], d['min'], d['max'])
                    yield (
                        d['name'],
                        IntegerField(
                            initial=d['default'],
                            min_value=d['min'] if not math.isinf(d['min']) else None,
                            max_value=d['max'] if not math.isinf(d['max']) else None,
                            help_text=description
                        ),
                    )

                elif d['type'] == 'str':
                    yield (
                        d['name'],
                        CharField(
                            initial=d['default'],
                            help_text=d['description']
                        ),
                    )

                elif d['type'] == 'double':
                    description = '%s (Range: %s &le; x &le; %s)' % (d['description'], d['min'], d['max'])
                    yield (
                        d['name'],
                        FloatField(
                            initial=d['default'],
                            min_value=d['min'] if not math.isinf(d['min']) else None,
                            max_value=d['max'] if not math.isinf(d['max']) else None,
                            help_text=description
                        ),
                    )

            except Exception:
                pass

    def fields_from_group_desc(desc, layout):
        parameters = desc['parameters'] if isinstance(desc, dict) else desc
        for f in fields_from_param_desc(parameters):
            layout['parameters'].append(f[0])
            yield f

        if isinstance(desc, list):
            return

        groups = sorted(desc['groups'].values(), key=lambda g: g['id'])
        for g in groups:
            tab = string.capwords(g['name'].replace('_', ' '))
            layout['groups'][tab] = {
                'parameters': [],
                'groups': OrderedDict(),
            }
            for f in fields_from_group_desc(g, layout['groups'][tab]):
                yield f

    def fields_from_config_msg(config_msg, layout):
        try:
            bools = config_msg['bools']
            for c in bools:
                layout['parameters'].append(c['name'])
                yield (c['name'], BooleanField(required=False))
        except Exception:
            pass

        try:
            ints = config_msg['ints']
            for c in ints:
                layout['parameters'].append(c['name'])
                yield (c['name'], IntegerField())
        except Exception:
            pass

        try:
            doubles = config_msg['doubles']
            for c in doubles:
                layout['parameters'].append(c['name'])
                yield (c['name'], FloatField())
        except Exception:
            pass

        try:
            strs = config_msg['strs']
            for c in strs:
                layout['parameters'].append(c['name'])
                yield (c['name'], CharField())
        except Exception:
            pass

    layout = {
        'parameters': [],
        'groups': OrderedDict(),
    }
    if isinstance(desc, (dict, list)):
        class_dict = dict(fields_from_group_desc(desc, layout))
    elif isinstance(config_msg, dict):
        class_dict = dict(fields_from_config_msg(config_msg, layout))
    else:
        class_dict = {}

    if not allow_protected:
        for n, f in class_dict.items():
            if not n.endswith('_'):
                f._kwargs['read_only'] = True
                f._kwargs['write_only'] = False
                f._kwargs['required'] = False
                class_dict[n] == type(f)(*f._args, **f._kwargs)

    class_dict[str('layout')] = serializers.ReadOnlyField(default=json.dumps(layout))

    meta_class = type(str('Meta'), tuple(), {
        'model': Parameter,
        'fields': tuple(chain(class_dict.keys(), ['key'])),
        'read_only_fields': ('key', ),
    })

    class_dict[str('LAYOUT')] = layout
    class_dict[str('Meta')] = meta_class

    return SerializerMetaclass(str('ParameterSerializer'), (ParameterSerializer, ), class_dict)


class InitialFieldInfoMixin(object):
    # initial - to provide dynamic_reconfigure default value to frontend.

    def get_field_info(self):
        return OrderedDict([
            ('initial', self.initial)
        ])


CharField = type(str('CharField'), (InitialFieldInfoMixin, serializers.CharField), {})
ChoiceField = type(str('CharField'), (InitialFieldInfoMixin, serializers.ChoiceField), {})
BooleanField = type(str('CharField'), (InitialFieldInfoMixin, serializers.BooleanField), {})
IntegerField = type(str('CharField'), (InitialFieldInfoMixin, serializers.IntegerField), {})
FloatField = type(str('CharField'), (InitialFieldInfoMixin, serializers.FloatField), {})


class MultipleChoiceField(serializers.MultipleChoiceField):

    def run_validation(self, data=None):
        value = super(MultipleChoiceField, self).run_validation(data=data)
        return ','.join(value)

    def to_representation(self, value):
        value = [v for v in value.split(',') if v]
        return super(MultipleChoiceField, self).to_representation(value)


class ClearableFileField(serializers.FileField):
    def __init__(self, accept=None, *args, **kwargs):
        self.accept = accept
        super(ClearableFileField, self).__init__(*args, **kwargs)

    def get_field_info(self):
        return OrderedDict([
            ('accept', self.accept)
        ])

    def to_internal_value(self, data):
        # form submit empty string.
        if data == '':
            return None

        # clear if send true
        if data == 'on':
            return True

        file_object = super(ClearableFileField, self).to_internal_value(data)
        if self.accept and file_object.size and file_object.content_type not in self.accept:
            self.fail('invalid')
        if ';' in file_object.name:
            # TODO: more verbose error info. (Illegal character in filename.)
            self.fail('invalid')

        return file_object


class ListField(serializers.ListField):

    def get_value(self, dictionary):
        if self.field_name in dictionary:
            val = dictionary.get(self.field_name, serializers.empty)
            if val == '':
                return serializers.empty

        return super(ListField, self).get_value(dictionary)


class CheckboxMultipleChoiceField(serializers.MultipleChoiceField):
    def get_field_info(self):
        return OrderedDict([
            ('type', 'checkbox multiple choice')
        ])


class TextareaCharField(serializers.CharField):
    def get_field_info(self):
        return OrderedDict([
            ('type', 'textarea string')
        ])


class AudioSerializerMixin(serializers.ModelSerializer):
    submit_edit = False
    MEDIA_FIELDS = ['alarm_media_', 'beep_media_'] + ['music_playlist_%d_files_' % i for i in range(1, 11)]

    media_root_ = serializers.HiddenField(default=default_storage.location)
    alarm_media_ = ClearableFileField(
        use_url=True,
        required=False,
        allow_empty_file=False,
        accept=['audio/x-wav', 'audio/wav'],
    )
    beep_media_ = ClearableFileField(
        use_url=True,
        required=False,
        allow_empty_file=False,
        accept=['audio/x-wav', 'audio/wav'],
    )


def audioserializer_factory(parameter_serializer_class):
    class_dict = {}

    # TODO: fix offline layout is not generated...
    # ensure offline will also show file field.
    for i in range(1, 11):
        class_dict['music_playlist_%d_files_' % i] = ListField(
            child=ClearableFileField(
                use_url=True,
                required=False,
                allow_empty_file=False,
                accept=['audio/mpeg'],
            ),
            required=False,
            allow_empty=True
        )

    # update layout for hidden field.
    layout = parameter_serializer_class.LAYOUT
    layout['parameters'] = [v for v in layout['parameters'] if v not in ['media_root_']]
    if 'General' in layout['groups']:
        layout['groups']['General']['parameters'] = [
            v
            for v in layout['groups']['General']['parameters']
            if v not in ['media_root_']
        ]

    if not layout['parameters'] and not layout['groups']:
        layout['parameters'] = AudioSerializerMixin.MEDIA_FIELDS

    class_dict[str('layout')] = serializers.ReadOnlyField(default=json.dumps(layout))

    # regen meta incase fields does not exist
    meta_class = type(str('Meta'), tuple(), {
        'model': Parameter,
        'fields': list(set(chain(
            AudioSerializerMixin.MEDIA_FIELDS,
            ['key', 'layout'],
            parameter_serializer_class.Meta.fields,
        ))),
        'read_only_fields': ('key', ),
    })
    class_dict[str('Meta')] = meta_class

    return SerializerMetaclass(str('AudioSerializer'), (AudioSerializerMixin, parameter_serializer_class), class_dict)


def permissionlistserializer_factory(change_change_permission=False, **kwargs):
    boot_codenames = [
        'start_agv05', 'stop_agv05',
        'soft_reboot', 'hard_reboot',
        'hot_reload', 'poweroff',
    ]
    panel_codenames = [
        'start_task_runner',
        'stop_task_runner',
        'pause_task_runner',
        'resume_task_runner',
        'start_app',
        'stop_app',
        'show_panel_side_menu',
        'show_panel_call_buttons',
        'show_panel_running_tasks',
        'show_panel_completed_tasks',
        'show_panel_all_modules',
        'show_panel_live_map',
        'show_panel_live_camera',
        'show_panel_health_status',
        'show_panel_live_monitor',
        'show_panel_live_manipulation',
        'show_system_clock',
        'show_system_icon',
        'show_popup_safety',
        'show_popup_user_own',
        'show_popup_user_public',
        'show_popup_user_private',
    ]
    operation_codenames = [
        'add_task',
        'abort_own_task', 'abort_task',
        'cancel_own_task', 'cancel_task',
        'prioritize_own_task', 'prioritize_task',
        'suspend_own_task', 'suspend_task',
        'resume_own_task', 'resume_task',
        'use_tasktemplate',
        'abort_transaction', 'cancel_transaction',
        'resume_transaction',
    ]
    system_codenames = kwargs.get('system_codenames', [
        'view_panel', 'view_system_panel',
        'view_agv_activities', 'view_completed_tasks',
        'view_log_files', 'view_help_content',
        'change_agv',
        'add_map', 'change_map', 'delete_map',
        'add_mapchangeset', 'change_mapchangeset',
        'change_parameter', 'change_protected_parameter', 'delete_parameter',
        'add_tasktemplate', 'change_tasktemplate', 'delete_tasktemplate',
        'add_webhook', 'change_webhook', 'delete_webhook',
        'backup_restore',
        'change_network',
        'change_datetime',
        'change_agvactivity', 'change_agv_activities_config',
        'change_license',
        'update_software',
    ])
    user_codenames = [
        'view_users', 'add_user', 'change_user', 'delete_user',
        'change_own_password',
        'add_group', 'change_group', 'delete_group',
        'change_permission',
    ]
    wifi_codenames = [
        'wifi_connect', 'wifi_adhoc',
    ]
    agv_n_group_codenames = boot_codenames + panel_codenames + operation_codenames + wifi_codenames
    group_only_codenames = system_codenames + user_codenames

    user_choices = User.objects.filter(username__in=['agv_panel', 'agv_panel_pin_protected']).values_list('pk', 'username')
    group_choices = Group.objects.all().values_list('pk', 'name')

    class_dict = {}

    perms = Permission.objects.filter(codename__in=agv_n_group_codenames)
    for perm in perms:
        class_dict['%s_0' % perm.codename] = CheckboxMultipleChoiceField(
            label=perm.name, choices=user_choices, initial=perm.user_set.all().values_list('pk', flat=True),
            required=False)
        class_dict['%s_1' % perm.codename] = CheckboxMultipleChoiceField(
            label=' ', choices=group_choices, initial=perm.group_set.all().values_list('pk', flat=True),
            required=False)

    perms = Permission.objects.filter(codename__in=group_only_codenames)
    for perm in perms:
        if not change_change_permission and perm.codename == 'change_permission':
            class_dict[str(perm.codename)] = CheckboxMultipleChoiceField(
                label=perm.name, choices=group_choices, default=perm.group_set.all().values_list('pk', flat=True),
                required=False, read_only=True)
            continue
        class_dict[str(perm.codename)] = CheckboxMultipleChoiceField(
            label=perm.name, choices=group_choices, initial=perm.group_set.all().values_list('pk', flat=True),
            required=False)

    layout = {
        'parameters': [],
        'groups': OrderedDict(),
    }
    groups = layout['groups']
    groups['AGV Boot'] = {
        'parameters': [tpl % cn for cn in boot_codenames for tpl in ['%s_0', '%s_1']],
        'groups': OrderedDict(),
    }
    groups['AGV Panel'] = {
        'parameters': [tpl % cn for cn in panel_codenames for tpl in ['%s_0', '%s_1']],
        'groups': OrderedDict(),
    }
    groups['Task Operation'] = {
        'parameters': [tpl % cn for cn in operation_codenames for tpl in ['%s_0', '%s_1']],
        'groups': OrderedDict(),
    }
    groups['System'] = {
        'parameters': system_codenames,
        'groups': OrderedDict(),
    }
    groups['Users'] = {
        'parameters': user_codenames,
        'groups': OrderedDict(),
    }
    groups['Wifi'] = {
        'parameters': [tpl % cn for cn in wifi_codenames for tpl in ['%s_0', '%s_1']],
        'groups': OrderedDict(),
    }

    class_dict[str('layout')] = serializers.ReadOnlyField(default=json.dumps(layout))
    class_dict[str('LAYOUT')] = layout

    def save(self, *args, **kwargs):
        for perm in Permission.objects.filter(codename__in=agv_n_group_codenames):
            perm.user_set = self.validated_data.get('%s_0' % perm.codename)
            perm.group_set = self.validated_data.get('%s_1' % perm.codename)

        for perm in Permission.objects.filter(codename__in=group_only_codenames):
            perm.group_set = self.validated_data.get(perm.codename)

    class_dict[str('save')] = save

    return SerializerMetaclass(str('PermissionListSerializer'), (serializers.Serializer, ), class_dict)


class WhiteLabelSerializer(serializers.Serializer):
    address_label = TextareaCharField(required=False)
    copyright_label = serializers.CharField(required=False)

    favicon_label = ClearableFileField(
        use_url=True,
        required=False,
        allow_empty_file=False,
        accept=['image/x-icon'],
    )
    logo_label = ClearableFileField(
        use_url=True,
        required=False,
        allow_empty_file=False,
        accept=['image/png'],
    )


class SerializerMetaclass(serializers.SerializerMetaclass):
    @classmethod
    def _get_declared_fields(cls, bases, attrs):
        # TODO: why they reversed this?.
        # HACK: ensure field override correctly.
        bases = list(reversed(bases))
        return super(SerializerMetaclass, cls)._get_declared_fields(bases, attrs)


class HwAppSerializer(serializers.Serializer):
    id = serializers.CharField()
    name = serializers.CharField()
    entry = serializers.SerializerMethodField()

    def get_entry(self, obj):
        id = obj['id']
        entry = obj['entry']
        if not id or not entry:
            return ''

        return reverse(
            'system:hw-app',
            request=self.context.get('request'),
            kwargs={
                'app_id': id,
                'asset': entry,
            }
        )
