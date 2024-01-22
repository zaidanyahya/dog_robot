#!/usr/bin/env python3
import os
import random
import rclpy
from rclpy.node import Node
from playsound import playsound

NODE_NAME = "random_sound"


class RandomSound(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()
        self.sound_files = ["dog1a.wav", "dog2a.wav", "dog3a.wav"]

        self.create_timer(5.0, self.random_sound_callback)
        full_path = os.path.realpath(__file__)
        self.sound_files_path = os.path.dirname(full_path)

    def random_sound_callback(self):
        sound_file = random.choice(self.sound_files)
        self.logger.info(f"Playing sound: {sound_file}")

        try:
            playsound(f"{self.sound_files_path}/{sound_file}")
        except Exception as e:
            self.logger.error(f"Error playing sound: {str(e)}")


def main():
    rclpy.init()

    node = RandomSound()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if "__name__" == "__main__":
    main()
