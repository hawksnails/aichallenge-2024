import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'site-packages'))

import casadi 
import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import PathWithLaneId, Trajectory, TrajectoryPoint


class PathToTrajectory(Node):

    def __init__(self):
        super().__init__('path_to_trajectory_node')
        self.pub_ = self.create_publisher(Trajectory, 'output', 10)
        self.sub_ = self.create_subscription(
            PathWithLaneId,
            'input',
            self.callback,
            10
        )

    def callback(self, msg: PathWithLaneId):
        trajectory = Trajectory()
        trajectory.header = msg.header
        for path_point_with_lane_id in msg.points:
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose = path_point_with_lane_id.point.pose
            trajectory_point.longitudinal_velocity_mps = path_point_with_lane_id.point.longitudinal_velocity_mps
            trajectory.points.append(trajectory_point)
        self.pub_.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = PathToTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

