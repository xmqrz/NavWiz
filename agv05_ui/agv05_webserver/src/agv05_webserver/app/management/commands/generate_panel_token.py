from __future__ import absolute_import
from __future__ import unicode_literals

from django.conf import settings as django_settings
from django.contrib.auth import get_user_model
from django.core.management.base import BaseCommand
from rest_framework.authtoken.models import Token
import os
import rospkg
import sys


class Command(BaseCommand):
    help = 'Generate token for agv panel user.'

    def handle(self, *args, **options):
        User = get_user_model()
        try:
            user_agv_panel = User.objects.get(username='agv_panel')
            user_agv_panel_pp = User.objects.get(username='agv_panel_pin_protected')
        except Exception:
            self.stderr.write('Cannot find user agv_panel in database. Have you applied the migrations?')
            sys.exit(1)

        token, created = Token.objects.get_or_create(user=user_agv_panel_pp)
        if not created:
            # Regenerate a new key
            token.delete()
            token.key = None
            token.save()

        token, created = Token.objects.get_or_create(user=user_agv_panel)
        if not created:
            # Regenerate a new key
            token.delete()
            token.key = None
            token.save()

        try:
            rospack = rospkg.RosPack()
            token_js = os.path.join(rospack.get_path('agv05_webapp'),
                'extension/token-x.js' if django_settings.TRACKLESS else 'extension/token.js')

            with open(token_js, 'w') as f:
                f.write('window.agvPanelToken%s = \'%s\';' % (
                    'X' if django_settings.TRACKLESS else '', token.key))

        except rospkg.ResourceNotFound as ex:
            self.stderr.write('Cannot find package agv05_webapp: %s' % ex)
            sys.exit(1)
        except Exception as ex:
            self.stderr.write('Error: %s' % ex)
            sys.exit(1)

        self.stdout.write('Successfully created .token.js for authentication use by agv panel app.')
