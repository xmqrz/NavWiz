from __future__ import absolute_import
from __future__ import unicode_literals

from celery.schedules import crontab, schedule
from django.conf import settings as django_settings


BROKER_URL = 'redis://'
CELERY_ACCEPT_CONTENT = ['pickle']
CELERY_RESULT_BACKEND = 'redis://'
CELERYD_MAX_TASKS_PER_CHILD = 10000

CELERYBEAT_SCHEDULER = 'djcelery.schedulers.DatabaseScheduler'
CELERYBEAT_SCHEDULE = {
    'archive-agv-activities': {
        'task': 'agv05_webserver.system.tasks.archive_agv_activities.archive_agv_activities',
        'schedule': crontab(minute='10', hour='*/3'),
        'options': {
            'expires': 9000,  # 2.5 hours
        },
    },
    'archive-tasks': {
        'task': 'agv05_webserver.system.tasks.archive_tasks.archive_tasks',
        'schedule': crontab(minute='10', hour='*/3'),
        'options': {
            'expires': 9000,  # 2.5 hours
        },
    },
    'build-agv-statistics': {
        'task': 'agv05_webserver.system.tasks.build_agv_statistics.build_agv_statistics',
        'schedule': crontab(minute='5', hour='*'),
        'options': {
            'expires': 60 * 45,  # 45 minutes
        },
    },
    'build-task-statistics': {
        'task': 'agv05_webserver.system.tasks.build_task_statistics.build_task_statistics',
        'schedule': crontab(minute='0', hour='*'),
        'options': {
            'expires': 60 * 45,  # 45 minutes
        },
    },
    'maintain-fms-pairing': {
        'task': 'agv05_webserver.%s.tasks.maintain_fms_pairing.maintain_fms_pairing' % (
            'system' if not django_settings.TRACKLESS else 'systemx'),
        'schedule': schedule(run_every=30),
        'options': {
            'expires': 20,
        },
    },
    'build-dashboard-statistics': {
        'task': 'agv05_webserver.system.tasks.build_dashboard_stats.build_dashboard_stats',
        'schedule': crontab(minute='0', hour='*'),
        'options': {
            'expires': 60 * 30,  # 30 minutes
        },
    },
    'update-license': {
        'task': 'agv05_webserver.system.tasks.update_license.trigger',
        'schedule': crontab(minute='15', hour='*'),
        'options': {
            'expires': 60 * 45,  # 45 minutes
        },
    },
    'cleanup-download-log': {
        'task': 'agv05_webserver.system.tasks.log_bundler.cleanup_log',
        'schedule': crontab(minute='20', hour='*/4'),
        'options': {
            'expires': 9000,  # 2.5 hours
        },
    },
}
CELERYBEAT_MAX_LOOP_INTERVAL = 300
