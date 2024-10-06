import math


class DiffDrive:
    def __init__(self, wheel_radius, wheel_base):
        """
        コンストラクタ
        wheel_radius: 車輪の半径（m）
        wheel_base: 左右の車輪間の距離（m）
        """
        self.wheel_radius = wheel_radius  # 車輪の半径
        self.wheel_base = wheel_base  # 車輪間の距離

    def calculate_velocity(self, left_wheel_speed, right_wheel_speed):
        """
        左右の車輪速度（m/s）から直線速度と角速度を計算する
        left_wheel_speed: 左車輪の速度（m/s）
        right_wheel_speed: 右車輪の速度（m/s）
        """
        # 直線速度 (linear velocity)
        linear_velocity = (left_wheel_speed + right_wheel_speed) / 2

        # 角速度 (angular velocity)
        angular_velocity = (right_wheel_speed - left_wheel_speed) / self.wheel_base

        return linear_velocity, angular_velocity

    def calculate_wheel_speeds(self, linear_velocity, angular_velocity):
        """
        直線速度と角速度から左右の車輪の速度を計算する
        linear_velocity: 直線速度（m/s）
        angular_velocity: 角速度（rad/s）
        """
        # 左右の車輪の速度 (m/s) を計算
        left_wheel_speed = linear_velocity - (angular_velocity * self.wheel_base) / 2
        right_wheel_speed = linear_velocity + (angular_velocity * self.wheel_base) / 2

        return left_wheel_speed, right_wheel_speed


# 使用例
if __name__ == "__main__":
    # 車輪の半径が0.1m、車輪間の距離が0.5mの場合
    robot = DiffDrive(wheel_radius=0.1, wheel_base=0.5)

    # 左右の車輪の直線速度 (m/s)
    left_speed = 1.0  # 左車輪
    right_speed = 1.5  # 右車輪

    # 直線速度と角速度を計算
    linear_vel, angular_vel = robot.calculate_velocity(left_speed, right_speed)
    print(f"直線速度: {linear_vel} m/s, 角速度: {angular_vel} rad/s")

    # 直線速度1.0 m/s, 角速度0.5 rad/sに対して左右の車輪速度を計算
    left_wheel, right_wheel = robot.calculate_wheel_speeds(linear_vel, angular_vel)
    print(f"左車輪の速度: {left_wheel} m/s, 右車輪の速度: {right_wheel} m/s")
