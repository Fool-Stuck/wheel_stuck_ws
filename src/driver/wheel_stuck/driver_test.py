import can
import time

class MotorController:
    def __init__(self, motor_id, max_speed_mps, wheel_diameter):
        """
        motor_id: モーターのCAN ID
        max_speed_mps: モーターの最大速度 [m/s]
        wheel_diameter: ホイールの直径 [m]（モーターが車輪に接続されている場合）
        """
        self.motor_id = motor_id
        self.max_speed_mps = max_speed_mps
        self.wheel_circumference = 3.1416 * wheel_diameter  # 円周を計算
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')  # CANインターフェース
        
        # 初期化: モーターを初期化し、速度制御モードに切り替える
        self.initialize_motor()
        time.sleep(1)
        self.set_mode_velocity_control()
        
    def initialize_motor(self):
        """
        モーターを初期化するためのCANメッセージを送信
        必要な初期化手順に従い、モーターをリセットや通信設定を行う
        """
        # ID 1,2 をenableに
        init_command = [0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
          # init_command = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # 初期化コマンド
        msg = can.Message(
            arbitration_id=0x105,
            data=init_command,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            print("モーターを初期化しました。")
        except can.CanError as e:
            print(f"モーター初期化に失敗しました: {e}")

    def mps_to_rpm(self, speed_mps):
        """
        m/s から RPM (回転数) に変換する
        """
        # ホイールの円周を使って速度[m/s]を回転数[rpm]に変換
        rpm = (speed_mps / self.wheel_circumference) * 60
        return int(rpm)

    def set_speed(self, speed_mps):
        """
        指定された速度[m/s]をモーターに送信する
        speed_mps: 速度 [m/s]
        """
        if abs(speed_mps) > self.max_speed_mps:
            raise ValueError("指定された速度が最大速度を超えています！")

        # m/s を RPM に変換
        rpm = self.mps_to_rpm(speed_mps)

        # CAN メッセージを構築して送信
        # ID 1 にrpm を送信
        
        speed_command = [(rpm >> 8) & 0xFF, rpm & 0xFF, (rpm >> 8) & 0xFF, rpm & 0xFF, 0x00, 0x00, 0x00, 0x00]  # 速度のデータを含む
        msg = can.Message(
            arbitration_id=0x32,
            data=speed_command,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            print(f"速度 {speed_mps} m/s をモーターに設定しました（{rpm} RPM）")
        except can.CanError as e:
            print(f"速度設定に失敗しました: {e}")

    def stop_motor(self):
        """
        モーターを停止する
        1. 速度を0に設定
        2. トルクを0に設定
        """
        # 速度を0に設定
        self.set_speed(0)

        # トルクを0に設定
        self.set_mode_disable()

        print("モーターを停止しました。")

    def set_mode_velocity_control(self):
        """
        モーターを速度制御モードに設定する
        """
        # モーターを速度制御モードに切り替えるCANメッセージを送信
        velocity_control_command = [0x02, 0x02, 0x00, 0x00, 0x00f, 0x00, 0x00, 0x00]  # 速度制御モードの設定値
        
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
        モーターをトルク制御モードに設定する
        """
        # モーターをトルク制御モードに切り替えるCANメッセージを送信
        torque_control_command = [0x09, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # トルク制御モードの設定値
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

    def shutdown(self):
        """
        CANバスを閉じる
        """
        self.bus.shutdown()
        print("CANバスをシャットダウンしました。")

# 使用例:
if __name__ == "__main__":
    motor_controller = MotorController(motor_id=1, max_speed_mps=1.6, wheel_diameter=0.185)
    motor_controller.set_speed(1.1)  # 速度を 1.1 m/s に設定

    time.sleep(3)
    
    motor_controller.stop_motor()    # モーターを停止
    
    motor_controller.shutdown()      # CANバスを閉じる
