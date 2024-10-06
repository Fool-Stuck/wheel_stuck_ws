import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('twist_subscriber')
        
        # Twistメッセージを購読するサブスクライバを作成
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # 購読するトピック名（デフォルトは cmd_vel）
            self.listener_callback,
            10)  # キューサイズ

        self.subscription  # 変数を保持しておく

    def listener_callback(self, msg):
        # Twistメッセージからlinearとangularの情報を取得
        linear = msg.linear
        angular = msg.angular
        
        # コンソールに出力
        self.get_logger().info(f'Linear: x={linear.x}, y={linear.y}, z={linear.z}')
        self.get_logger().info(f'Angular: x={angular.x}, y={angular.y}, z={angular.z}')


def main(args=None):
    # rclpyの初期化
    rclpy.init(args=args)

    # ノードを作成し、サブスクライバを開始
    twist_subscriber = TwistSubscriber()

    # ノードをスピンして実行（Ctrl+Cで停止）
    rclpy.spin(twist_subscriber)

    # ノードをシャットダウン
    twist_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
