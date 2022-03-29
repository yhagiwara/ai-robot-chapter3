import rclpy
import rclpy.node
from ai_robot_book_interfaces.srv import StringCommand

from gtts import gTTS

import speech_recognition as sr
import pyaudio
from io import BytesIO
from mpg123 import Mpg123, Out123

import threading

class SpeechClient(rclpy.node.Node):
    def __init__(self):
        super().__init__('speech_client')

        self.get_logger().info('音声認識を起動します')

        self.client = self.create_client(StringCommand, '/speech')

        self.init_rec = sr.Recognizer()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービスが起動していません ...')

        self.request = StringCommand.Request()

    def send_request(self):
        text = self.recognition()
        self.request.command = text
        return self.client.call_async(self.request)

    def recognition(self):
        text = ''

        with sr.Microphone() as source:
            audio_data = self.init_rec.record(source, duration=5)
            self.get_logger().info("音声認識中です・・・")

            try:
                text = self.init_rec.recognize_google(audio_data)
                self.get_logger().info(text)

            except sr.UnknownValueError:
                pass

        self.get_logger().info(f'認識した音声は "{text}" です')

        return text



class SpeechServer(rclpy.node.Node):
    def __init__(self):
        super().__init__('speech_server')

        self.get_logger().info('音声合成ノードを起動します')

        self.period = 5.0

        self.service = self.create_service(StringCommand, '/speech', self.command_callback)

        self.lang = 'ja-JP'
        self.mp3 = Mpg123()
        # self.out = Out123()

    def command_callback(self, request, response):
        self.get_logger().info(f'受けとったメッセージは "{request.command}"')

        self.speech_synthesis(request.command)

        response.answer = 'done'
        return response

    def speech_synthesis(self, text):
        self.get_logger().info('音声合成')

        tts = gTTS(text, lang=self.lang[:2])
        fp = BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)
        self.mp3.feed(fp.read())

        for frame in self.mp3.iter_frames(self.out.start):
            self.out.play(frame)

def run_server():
    speech_server = SpeechServer()

    try:
        rclpy.spin(speech_server)
    except:
        rclpy.shutdown()


def main():
    rclpy.init()

    server_thread = threading.Thread(target=run_server)
    server_thread.start()

    speech_client = SpeechClient()
    speech_client.send_request()


if __name__ == "__main__":
    main()
