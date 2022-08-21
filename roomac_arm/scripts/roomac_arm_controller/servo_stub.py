import rospy
import utils


class ServoStub(object):
    def __init__(
        self, name, cmd_topic, zero_angle_signal, angle_to_signal_scale_factor, cmd_type
    ):
        self._cmd_sub = rospy.Subscriber(
            cmd_topic,
            cmd_type,
            self._cmd_cb,
        )
        self.name = name

        self.signal = zero_angle_signal

        self._zero_angle_signal = zero_angle_signal
        self._angle_to_signal_scale_factor = angle_to_signal_scale_factor

    def calculate_position(self):
        if not self.signal:
            raise RuntimeError(
                "Servo " + self.name + " not initialized (signal wasn't yet received)"
            )

        return (
            self.signal - self._zero_angle_signal
        ) / self._angle_to_signal_scale_factor

    def _cmd_cb(self, msg):
        self.signal = msg.signal


class GripperServoStub(ServoStub):
    def __init__(
        self, name, cmd_topic, zero_angle_signal, angle_to_signal_scale_factor, cmd_type
    ):
        super(GripperServoStub, self).__init__(
            name, cmd_topic, zero_angle_signal, angle_to_signal_scale_factor, cmd_type
        )

        self.a = rospy.get_param("~gripper_angle_to_distance_a", -0.011875)
        self.b = rospy.get_param("~gripper_angle_to_distance_b", 0.007375)

    def calculate_position(self):
        angle = super(GripperServoStub, self).calculate_position()
        return utils.linear_transform_angle_to_dist(self.a, self.b, angle)
