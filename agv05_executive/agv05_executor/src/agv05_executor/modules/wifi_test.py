from __future__ import absolute_import
from __future__ import unicode_literals

from datetime import datetime
import codecs
import diagnostic_msgs.msg
import os
import rospy

from .module import Module


class WifiTest(Module):
    id = 'wifi-test'

    def __init__(self, *args, **kwargs):
        super(WifiTest, self).__init__(*args, **kwargs)

    def start(self):
        self.recording = False
        self.log_file = None

        self.__diagnostic_sub = rospy.Subscriber('/diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.handle_diagnostics, queue_size=1)

    def stop(self):
        self._stop_recording()
        self.__diagnostic_sub.unregister()

    def handle_in_pipe(self, data):
        try:
            cmd = data['command']
            if cmd == 'status':
                self._send_updates()
            elif cmd == 'start_recording':
                self._start_recording()
            elif cmd == 'stop_recording':
                self._stop_recording()
        except KeyError as ex:
            rospy.logerr('[WifiTest] invalid input: %s', ex)
        except ValueError as ex:
            rospy.logerr('[WifiTest] invalid input: %s', ex)
        except Exception as ex:
            rospy.logerr('Exception: %s', ex)

    def handle_diagnostics(self, msg):
        d = dict()
        d['timestamp'] = msg.header.stamp.secs

        for status in msg.status:
            if status.name[status.name.find('/', 1):] == '/Wifi Manager/Status':
                break
        else:
            return

        d['wifi'] = [{'key': kv.key, 'value': kv.value} for kv in status.values]
        if self.robot.fms_manager:
            d['fms'] = [{
                'key': 'Status',
                'value': self.robot.fms_manager._status.replace('\n', ' '),
            }, {
                'key': 'Broken',
                'value': '%s' % self.robot.fms_manager._status_broken,
            }]

        self.out({
            'command': 'data',
            'data': d,
        })

        # Write to file
        if not self.recording:
            return

        if self.first_line:
            w = '","'.join(['Wifi: %s' % kv['key'] for kv in d['wifi']])
            f = '","'.join(['DFleet: %s' % kv['key'] for kv in d.get('fms', [])])
            s = '"Time","%s","%s"\n' % (w, f)
            self.log_file.write(s)
            self.first_line = False

        t = datetime.fromtimestamp(d['timestamp'])
        w = '","'.join([kv['value'] for kv in d['wifi']])
        f = '","'.join([kv['value'] for kv in d.get('fms', [])])
        s = '"%s","%s","%s"\n' % (t, w, f)
        self.log_file.write(s)

    def _start_recording(self):
        if self.recording:
            return

        # Create recording file
        folder = os.path.join(os.environ['HOME'], '.ros/wifi')
        try:
            os.makedirs(folder)
        except OSError:
            pass

        file_path = os.path.join(folder, datetime.now().strftime('%Y%m%d-%H%M%S.csv'))
        self.log_file = codecs.open(file_path, 'w', 'utf-8')
        self.log_file.write('\ufeff')  # Unicode BOM
        self.first_line = True
        self.recording = True
        self._send_updates()

    def _stop_recording(self):
        if not self.recording:
            return

        # Save recording file
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        self.recording = False
        self._send_updates()

    def _send_updates(self):
        self.out({
            'command': 'status',
            'status': 'recording' if self.recording else 'idle',
        })
