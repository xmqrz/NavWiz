from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_webserver.systemx.models import Heading

from .transition_trigger import TransitionTriggerValidator


class TransitionTriggerXValidator(TransitionTriggerValidator):
    HEADING_NA = Heading.NA
    HEADING_WARNING = 'headingless'
