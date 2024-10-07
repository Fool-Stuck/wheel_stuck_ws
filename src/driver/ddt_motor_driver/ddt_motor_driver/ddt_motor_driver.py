import math
import time

import can


class DDTMotorDriver:
    def __init__(self, max_speed_mps, wheel_diameter, max_acceleration_mps2):
        """
        max_speed_mps: モーターの最大速度 [m/s]
        wheel_diameter: ホイールの直径 [m]（モーターが車輪に接続されている場合）
        max_acceleration_mps2: 最大加速度 [m/s^2]
        """
        self.max_speed_mps = max_speed_mps
        self.wheel_circumference = math.pi * wheel_diameter  # 円周を計算
        self.max_acceleration_mps2 = max_acceleration_mps2  # 最大加速度
        self.current_speeds = [0.0] * 8  # 現在の各モーターの速度[m/s]
        self.bus = can.interface.Bus(channel="can0", interface="socketcan")  # CANインターフェース

        # 初期化: モーターを初期化し、速度制御モードに切り替える
        self.initialize_motor()

    def initialize_motor(self):
        self.set_mode_enable()
        self.set_mode_velocity_control()

    def mps_to_rpm(self, speed_mps):
        """
        m/s から RPM (回転数) に変換する
        """
        rpm = (speed_mps / self.wheel_circumference) * 60
        return int(rpm)

    def set_mode_enable(self):
        # ID 1 to 8 をenableに
        init_command = [0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A]
        msg = can.Message(arbitration_id=0x105, data=init_command, is_extended_id=False)
        try:
            self.bus.send(msg)
            time.sleep(1)
            print("モーターを初期化しました。")
        except can.CanError as e:
            print(f"モーター初期化に失敗しました: {e}")

    def set_mode_velocity_control(self):
        """
        モーターを速度制御モードに設定する
        """
        velocity_control_command = [0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02]
        msg = can.Message(arbitration_id=0x105, data=velocity_control_command, is_extended_id=False)
        try:
            self.bus.send(msg)
            print("モーターを速度制御モードに設定しました。")
        except can.CanError as e:
            print(f"速度制御モード設定に失敗しました: {e}")

    def set_mode_disable(self):
        """
        モーターをDisableにする
        """
        torque_control_command = [0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09]
        msg = can.Message(arbitration_id=0x105, data=torque_control_command, is_extended_id=False)
        try:
            self.bus.send(msg)
            print("モーターをトルク制御モードに設定しました。")
        except can.CanError as e:
            print(f"トルク制御モード設定に失敗しました: {e}")

    def set_speed(self, target_speeds_mps):
        """
        速度指令を与える際に加速度制限を加える
        target_speeds_mps: 各モーターの目標速度 [m/s] の配列
        """
        if len(target_speeds_mps) != 8:
            raise ValueError("速度指令はID 1〜8までの設定を配列で与える必要があります。")

        rpm_list = [0.0] * 8

        for i, target_speed_mps in enumerate(target_speeds_mps):
            """
            指定された速度[m/s]をモーターに送信する前に、加速度制限を考慮して速度を調整
            target_speed_mps: 目標速度 [m/s]
            """
            if abs(target_speed_mps) > self.max_speed_mps:
                raise ValueError("指定された速度が最大速度を超えています！")

            # 加速度制限を考慮して新しい速度を決定
            speed_diff = target_speed_mps - self.current_speeds[i]
            max_speed_change = (
                self.max_acceleration_mps2 * 0.1
            )  # 加速度制限に基づいた最大速度変化量（0.1秒ごとに制御）

            if abs(speed_diff) > max_speed_change:
                # 加速度制限を超えないように調整
                speed_diff = max_speed_change if speed_diff > 0 else -max_speed_change

            # 更新された速度
            self.current_speeds[i] += speed_diff
            rpm_list[i] = self.mps_to_rpm(self.current_speeds[i])

        # CAN メッセージを構築して送信
        speed_command0 = [
            (rpm_list[0] >> 8) & 0xFF,
            rpm_list[0] & 0xFF,
            (rpm_list[1] >> 8) & 0xFF,
            rpm_list[1] & 0xFF,
            (rpm_list[2] >> 8) & 0xFF,
            rpm_list[2] & 0xFF,
            (rpm_list[3] >> 8) & 0xFF,
            rpm_list[3] & 0xFF,
        ]
        speed_command1 = [
            (rpm_list[4] >> 8) & 0xFF,
            rpm_list[4] & 0xFF,
            (rpm_list[5] >> 8) & 0xFF,
            rpm_list[5] & 0xFF,
            (rpm_list[6] >> 8) & 0xFF,
            rpm_list[6] & 0xFF,
            (rpm_list[7] >> 8) & 0xFF,
            rpm_list[7] & 0xFF,
        ]

        msg0 = can.Message(arbitration_id=0x32, data=speed_command0, is_extended_id=False)
        msg1 = can.Message(arbitration_id=0x33, data=speed_command1, is_extended_id=False)

        try:
            self.bus.send(msg0)
            self.bus.send(msg1)
        except can.CanError as e:
            print(f"速度設定に失敗しました: {e}")

    def stop_motor(self):
        """
        モーターを停止する
        """
        self.set_speed([0, 0, 0, 0, 0, 0, 0, 0])  # 速度を0に設定
        self.set_mode_disable()  # モーターを無効にする
        print("モーターを停止しました。")

    def shutdown(self):
        """
        CANバスを閉じる
        """
        self.bus.shutdown()
        print("CANバスをシャットダウンしました。")


# 使用例:
# if __name__ == "__main__":
#     motor_controller = DDTMotorDriver(max_speed_mps=1.6, wheel_diameter=0.185, max_acceleration_mps2=0.5)
#     motor_controller.set_speed([1.1, -1.1, 0, 0, 0, 0, 0, 0])
#     time.sleep(3)
#     motor_controller.stop_motor()    # モーターを停止
#     motor_controller.shutdown()      # CANバスを閉じる