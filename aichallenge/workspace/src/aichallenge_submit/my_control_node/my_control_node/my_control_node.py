import rclpy
from rclpy.node import Node
##from tier4_control_msgs.msg import AckermannControlCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand
from rclpy.qos import QoSProfile

class ControlCommandModifier(Node):
    def __init__(self):
        super().__init__('control_command_modifier')

        # QoS設定
        qos_profile = QoSProfile(depth=10)

        # Subscriberの設定
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            qos_profile
        )

        # Publisherの設定
        self.publisher = self.create_publisher(
            AckermannControlCommand,
            'output/raw_control_cmd',
            qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received control command: {msg.lateral.steering_tire_angle}')
        
        # 受け取ったメッセージの値を2倍にする
        modified_msg = AckermannControlCommand()
        modified_msg.lateral.steering_tire_angle = msg.lateral.steering_tire_angle * 2

        # 変更したメッセージをパブリッシュ
        self.publisher.publish(modified_msg)
        self.get_logger().info(f'Published modified control command: {modified_msg.lateral.steering_tire_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlCommandModifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
