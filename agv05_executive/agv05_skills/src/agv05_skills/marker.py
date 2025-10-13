from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_executor.skill import Skill
from geometry_msgs.msg import Pose2D
import math
import rospy
import tf


class FindMarker(Skill):

    class Meta:
        name = 'Find Marker'
        params = [
            {
                'name': 'marker_type',
                'type': 'str',
                'description': 'Marker type and sensor id, eg: reflector__laser1, vmarker__laser3',
            },
            {
                'name': 'x',
                'type': 'Register',
                'description': 'Register to store the X offset (in millimeters)',
            },
            {
                'name': 'y',
                'type': 'Register',
                'description': 'Register to store the Y offset (in millimeters)',
            },
            {
                'name': 'yaw',
                'type': 'Register',
                'description': 'Register to store the yaw offset (in degrees)',
            },
            {
                'name': 'timeout',
                'type': 'int',
                'description': 'Timeout duration in seconds.',
                'default': 3,
                'min': -1,
                'max': 9999,
            },
        ]
        outcomes = ['Success', 'Fail']
        mutexes = ['base']

    def __init__(self, *args, **kwargs):
        super(FindMarker, self).__init__(*args, **kwargs)
        if self.timeout < 0:
            self.timeout = None

    def __str__(self):
        return 'Finding marker'

    def execute(self, ud):
        rospy.loginfo('%s', self)
        if self.preempt_requested():
            return 'Preempted'

        self.robot.base.publish_marker_type(self.marker_type)
        pose = self.get_marker_pose()
        self.robot.base.publish_marker_type('')

        if self.preempt_requested():
            return 'Preempted'
        elif pose is None:
            return 'Fail'

        self.x.value = int(pose.x * 1000)
        self.y.value = int(pose.y * 1000)
        self.yaw.value = int(math.degrees(pose.theta))
        rospy.loginfo('Found marker at (%d, %d, %d)', self.x.value, self.y.value, self.yaw.value)
        return 'Success'

    def get_marker_pose(self):
        start_time = rospy.get_time()
        while not self.preempt_requested():
            if self.timeout is not None:
                if rospy.get_time() - start_time > self.timeout:
                    break

            try:
                t = self.robot.base.tf_listener.getLatestCommonTime('base', 'marker')
                if start_time - t.to_sec() <= 0.5:  # ensure transform is recent
                    (trans, rot) = self.robot.base.tf_listener.lookupTransform('base', 'marker', rospy.Time())
                    rpy = tf.transformations.euler_from_quaternion(rot)
                    pose = Pose2D()
                    pose.x = trans[0]
                    pose.y = trans[1]
                    pose.theta = rpy[2]
                    return pose
            except Exception:
                pass

            rospy.sleep(0.2)
