# Standard Libraries
import os
import random

# ROS2 Libraries
import rclpy
from rclpy.node import Node

# Other Libraries
from playsound import playsound

# Constants
NODE_NAME = "random_sound"
TIMER_PERIOD = 5  # seconds


# ランダムな音声を再生するノード
class RandomSound(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()
        self.sound_files = ["dog1a.wav", "dog2a.wav", "dog3a.wav"]
        full_path = os.path.realpath(__file__)
        self.sound_files_path = os.path.dirname(full_path)

        # ５秒ごとにランダムな音声を再生するためのタイマー
        self.create_timer(TIMER_PERIOD, self.random_sound_callback)

    # タイマーコールバック関数
    def random_sound_callback(self):
        # ランダムな音声ファイルの選択
        sound_file = random.choice(self.sound_files)
        # ログに再生中の音声ファイルを出力
        self.logger.info(f"Playing sound: {sound_file}")

        try:
            # 音声ファイルのの再生
            playsound(f"{self.sound_files_path}/{sound_file}")
        except Exception as e:
            # ログにエラーを出力
            self.logger.error(f"Error playing sound: {str(e)}")


def main():
    rclpy.init()

    node = RandomSound()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if "__name__" == "__main__":
    main()
