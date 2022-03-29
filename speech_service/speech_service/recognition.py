import rclpy
import rclpy.node
from std_msgs.msg import String

import speech_recognition as sr
import pyaudio


class Recognition(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_recognition")

        self.get_logger().info('音声認識ノードを起動します')

        self.init_rec = sr.Recognizer()

        self.publisher = self.create_publisher(String, '/speech', 1)

    def recognition(self):
        text = ''

        with sr.Microphone() as source:
            audio_data = self.init_rec.record(source, duration=5)
            self.logger.info(f'音声認識を行います')

            try:
                text = self.init_rec.recognize_google(audio_data)
                self.logger.info(text)
                response.answer = text

            except sr.UnknownValueError:
                pass

        msg = String()
        msg.data = text
        self.logger.info(f'認識した音声 "{text}" をトピック名 /speech に公開します')

        self.publisher.publish(msg)


def main():
    rclpy.init()

    recognition_node = Recognition()

    recognition_node.recognition()
    recognition_node.destroy_node()

    rclpy.shutdown()
