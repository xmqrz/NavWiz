from __future__ import absolute_import
from __future__ import unicode_literals

from .agv import AgvViewSet
from .agv_statistic import AgvStatisticViewSet, AgvActivitiesYearPagination
from .app import AppViewSet
from .boot import BootViewSet
from .dashboard import DashboardViewSet
from .log import LogViewSet
from .map import MapViewSet
from .register import RegisterViewSet
from .task import TaskViewSet
from .task_statistic import TaskStatisticViewSet, TaskCompletedYearPagination
from .task_template import TaskTemplateViewSet
from .time import TimeViewSet
from .transaction import TransactionViewSet
from .variable import VariableViewSet
from .version import VersionViewSet
from .white_label import WhiteLabelViewSet, WhiteLabelFileDownload
from .wifi import WifiViewSet
