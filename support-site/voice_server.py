import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand

from gtts import gTTS
from subprocess import run, PIPE

import Levenshtein

import os
from time import sleep
import speech_recognition as sr
import pyaudio
from io import BytesIO
from mpg123 import Mpg123, Out123

class VoiceServer(rclpy.node.Node):
    def __init__(self):
        super().__init__("voice_server")

        self.get_logger().info("音声サーバーを起動しました")

        self.period = 5.0
        self.init_rec = sr.Recognizer()

        self.objects = ['bottle', 'cup']
        self.places = ['kitchen', 'living']

        self.service = self.create_service(StringCommand, '/speech_server/wake_up', self.command_callback)

        self.lang = 'en'
        self.mp3 = Mpg123()
        self.out = Out123()

    def command_callback(self, request, response):

        self.synthesis('I\'m ready.')
        self.get_logger().info('I\'m ready.')

        text = None
        while text is None:
            text = self.recognition()

        target_object, target_palce = self.search_object_and_place(text)

        self.synthesis(f'I will go to the {target_palce} and grab a {target_object}')
        self.get_logger().info(f'I will go to the {target_palce} and grab a {target_object}')


        if (target_object is not None) and (target_palce is not None):
            response.answer = f'{target_palce},{target_object}'
            return response
        else:
            response.answer = 'failed'
            return response

    def recognition(self):

        with sr.Microphone() as source:
            audio_data = self.init_rec.record(source, duration=5)
            self.get_logger().info(f'音声認識を行います')

            try:
                text = self.init_rec.recognize_google(audio_data)
                self.get_logger().info(text)

            except sr.UnknownValueError:
                pass

        self.get_logger().info(f'認識したテキストは "{text}" です')

        return text

    def search_object_and_place(self, text):

        self.get_logger().info(f'受けとったテキスト "{text}"')

        target_object = None
        target_place = None

        for _object in self.objects:
            if _object in text:
                target_object = _object

        for _place in self.places:
            if _place in text:
                target_place = _place

        return target_object, target_place

    def synthesis(self, text):

        self.get_logger().info('音声合成')

        tts = gTTS(text, lang=self.lang[:2])
        fp = BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)
        self.mp3.feed(fp.read())

        for frame in self.mp3.iter_frames(self.out.start):
            self.out.play(frame)


def main():
    rclpy.init()

    voice_server = VoiceServer()

    rclpy.spin(voice_server)
    voice_server.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
