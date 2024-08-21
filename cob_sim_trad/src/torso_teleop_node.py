#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TeleopTorso(Node):

    def __init__(self):
        super().__init__('teleop_torso')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)

        # Initial positions
        self.torso_2_position = 0.0
        self.torso_3_position = 0.0

        # Control parameters
        self.position_increment = 0.01  # Incremental step for the position change
        self.rate = self.create_rate(10)  # 10 Hz

    def joy_callback(self, msg):
        # L1 button is usually index 4
        l1_button_pressed = msg.buttons[4] == 1

        if l1_button_pressed:
            # Right joystick vertical axis is usually index 4
            axis_value = msg.axes[4]

            # Increment or decrement the torso_3 position based on joystick input
            if axis_value != 0.0:
                self.torso_3_position += self.position_increment * axis_value

            # Create JointTrajectory message
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['torso_2_joint', 'torso_3_joint']
            point = JointTrajectoryPoint()
            point.positions = [self.torso_2_position, self.torso_3_position]
            point.time_from_start.sec = 1
            traj_msg.points.append(point)

            # Publish JointTrajectory message
            self.publisher_.publish(traj_msg)
            self.get_logger().info(f'Publishing: {traj_msg}')

def main(args=None):
    rclpy.init(args=args)
    teleop_torso = TeleopTorso()
    rclpy.spin(teleop_torso)
    teleop_torso.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
