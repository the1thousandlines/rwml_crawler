import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_interfaces.action import TrialAction
from std_srvs.srv import Empty
import sys
import time
import serial
import numpy as np


class TrialActionActionServer(Node):

    def __init__(self):
        super().__init__('TrialAction_action_server')
        self.srv = self.create_service(Empty, 'reset_env', self.reset_env_callback)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        time.sleep(1)
        self.joint_states = np.array([0.0, 0.0])
        self.joint_limits = [(0, 70), (0, 90)]
        self.update_states()
        self._action_server = ActionServer(
            self,
            TrialAction,
            'move',
            self.execute_callback)

    def update_states(self):
        for i in range(len(self.joint_states)):
            deg = int(self.joint_limits[i][1]*self.joint_states[i] + self.joint_limits[i][0])
            tw = "%d %d\n" % (i, deg)
            # print("writing:", tw)
            self.ser.write(bytes(tw, 'utf-8'))
            time.sleep(0.03)
            res = self.ser.read_all()
            # print("ANS:", res)
            self.ser.flushOutput()
            self.ser.flushInput()

    def reset_env_callback(self, request, response):
        self.joint_states = np.array([0.5, 0.0])
        self.update_states()
        time.sleep(0.5)
        print("reset")
        self.ser.write(bytes("6 0\n", 'utf-8'))
        time.sleep(1)
        self.ser.write(bytes("6 180\n", 'utf-8'))
        self.ser.flushOutput()
        self.ser.flushInput()
        time.sleep(0.1)
        self.joint_states = np.array([0.0, 0.0])
        self.update_states()
        time.sleep(0.5)
        print("reset complete")
        return response

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        joint = goal_handle.request.joint
        direct = goal_handle.request.direction
        self.get_logger().info("joint: %d, dir: %d" % (joint, direct))
        feedback_msg = TrialAction.Feedback()
        feedback_msg.current = self.joint_states[joint]
        goal_handle.publish_feedback(feedback_msg)
        # self.ser.write('0 0')

        for i in range(1):
            # time.sleep(0.1)
            self.joint_states[joint] += 0.15*direct
            self.joint_states = np.clip(self.joint_states, [0, 0], [1, 1])
            self.update_states()
            feedback_msg.current = self.joint_states[joint]
            goal_handle.publish_feedback(feedback_msg)

        # self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

        goal_handle.succeed()

        result = TrialAction.Result()
        result.final = self.joint_states[joint]
        print("Done, returning", self.joint_states)
        return result


def main(args=None):
    print(sys.executable)
    print(sys.version)
    rclpy.init(args=args)

    fibonacci_action_server = TrialActionActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
