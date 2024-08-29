from autoware_auto_planning_msgs.msg import PathWithLaneId, Trajectory, TrajectoryPoint
import rclpy

class PathToTrajectory(rclpy.Node):
    def __init__(self):
        super().__init__('path_to_trajectory_node')
        self.pub_ = self.create_publisher(Trajectory, 'output', 1)
        self.sub_ = self.create_subscription(PathWithLaneId, 'input', self.callback, 1)

    def callback(self, msg):
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
    path_to_trajectory = PathToTrajectory()
    rclpy.spin(path_to_trajectory)
    rclpy.shutdown()

if __name__ == '__main__':
    print("Starting casadi_planner")
    main()
