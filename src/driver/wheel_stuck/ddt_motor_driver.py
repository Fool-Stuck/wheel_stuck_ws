import can
import time
import math

class DDTMotorDriver:
    def __init__(self, max_speed_mps, wheel_diameter):
        """
        max_speed_mps: モーターの最大速度 [m/s]
        wheel_diameter: ホイールの直径 [m]（モーターが車輪に接続されている場合）
        """
        self.max_speed_mps = max_speed_mps
        self.wheel_circumference = math.pi * wheel_diameter  # 円周を計算
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')  # CANインターフェース
        
        # 初期化: モーターを初期化し、速度制御モードに切り替える
        self.initialize_motor()

    def initialize_motor(self):
        self.set_mode_enable()
        self.set_mode_velocity_control()

    def mps_to_rpm(self, speed_mps):
        """
        m/s から RPM (回転数) に変換する
        """
        # ホイールの円周を使って速度[m/s]を回転数[rpm]に変換
        rpm = (speed_mps / self.wheel_circumference) * 60
        return int(rpm)

    def set_mode_enable(self):
        # ID 1 to 8 をenableに
        init_command = [0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A]
        msg = can.Message(
            arbitration_id=0x105,
            data=init_command,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            time.sleep(1)
            print("モーターを初期化しました。")
        except can.CanError as e:
            print(f"モーター初期化に失敗しました: {e}")

    def set_mode_velocity_control(self):
        """
        モーターをFeedback速度制御モードに設定する
        """
        # モーターを速度制御モードに切り替えるCANメッセージを送信
        velocity_control_command = [0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02]  # 速度制御モードの設定値
        
        msg = can.Message(
            arbitration_id=0x105,
            data=velocity_control_command,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            print("モーターを速度制御モードに設定しました。")
        except can.CanError as e:
            print(f"速度制御モード設定に失敗しました: {e}")

    def set_mode_disable(self):
        """
        モーターをDisableにする
        """
        # モーターをトルク制御モードに切り替えるCANメッセージを送信
        torque_control_command = [0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09]  # トルク制御モードの設定値
        msg = can.Message(
            arbitration_id=0x105,
            data=torque_control_command,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            print("モーターをトルク制御モードに設定しました。")
        except can.CanError as e:
            print(f"トルク制御モード設定に失敗しました: {e}")

            
    def set_speed(self, speed_mpss):

        if len(speed_mpss) != 8:
            raise ValueError("Input Array size error : 速度指令はID 1〜8までの設定を配列で与える必要があります．")

        rpms = [0.0]*8
        
        for i, speed_mps in enumerate(speed_mpss):
            """
            指定された速度[m/s]をモーターに送信する
            speed_mps: 速度 [m/s]
            """
            if abs(speed_mps) > self.max_speed_mps:
                raise ValueError("指定された速度が最大速度を超えています！")

            # m/s を RPM に変換
            rpms[i] = self.mps_to_rpm(speed_mps)

        # CAN メッセージを構築して送信
        speed_command0 = [(rpms[0] >> 8) & 0xFF, rpms[0] & 0xFF, (rpms[1] >> 8) & 0xFF, rpms[1] & 0xFF, (rpms[2] >> 8) & 0xFF, rpms[2] & 0xFF, (rpms[3] >> 8) & 0xFF, rpms[3] & 0xFF]  # 速度のデータを含む
        speed_command1 = [(rpms[4] >> 8) & 0xFF, rpms[4] & 0xFF, (rpms[5] >> 8) & 0xFF, rpms[5] & 0xFF, (rpms[6] >> 8) & 0xFF, rpms[6] & 0xFF, (rpms[7] >> 8) & 0xFF, rpms[7] & 0xFF]  # 速度のデータを含む
        msg0 = can.Message(
            arbitration_id=0x32,
            data=speed_command0,
            is_extended_id=False
        )
        msg1 = can.Message(
            arbitration_id=0x33,
            data=speed_command1,
            is_extended_id=False
        )

        try:
            self.bus.send(msg0)
            self.bus.send(msg1)
        except can.CanError as e:
            print(f"速度設定に失敗しました: {e}")

    def stop_motor(self):
        """
        モーターを停止する
        1. 速度を0に設定
        2. トルクを0に設定
        """
        # 速度を0に設定
        self.set_speed([0,0,0,0,0,0,0,0])

        # motorをdisabledに
        self.set_mode_disable()

        print("モーターを停止しました。")

    def shutdown(self):
        """
        CANバスを閉じる
        """
        self.bus.shutdown()
        print("CANバスをシャットダウンしました。")

# 使用例:
if __name__ == "__main__":
    motor_controller = DDTMotorDriver(max_speed_mps=1.6, wheel_diameter=0.185)
    motor_controller.set_speed([1.1,-1.1,0,0,0,0,0,0]) 
    time.sleep(3)
    motor_controller.stop_motor()    # モーターを停止
    motor_controller.shutdown()      # CANバスを閉じる
