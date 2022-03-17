import rclpy
import rclpy.node
from ai_robot_book_interfaces.srv import StringCommand

from gtts import gTTS
from subprocess import run, PIPE

import Levenshtein

import os
from time import sleep
import speech_recognition as sr
import pyaudio
from io import BytesIO
from mpg123 import Mpg123, Out123


class SpeechOfBringMe(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_recognition")

        self.logger = self.get_logger()
        self.logger.info("Start speech recognition")

        self.period = 5.0
        self.init_rec = sr.Recognizer()

        self.Questions = {"Bring me a bottle from kitchen":"Ok, I will"}

        self.service = self.create_service(StringCommand, '/speech/command', self.command_callback)

        self.lang = 'ja-JP'
        self.mp3 = Mpg123()
        self.out = Out123()


    def command_callback(self, request, response):

        text = None

        if request.command == 'ask object':
            pass

        elif request.command == 'ask place':
            pass

        while text is None:
            text = self.recognition()

        answer = self.select_answer(text)

        self.speech_synthesis(answer)

        response.answer = answer
        return response

    def recognition(self):

        with sr.Microphone() as source:
            audio_data = self.init_rec.record(source, duration=5)
            self.logger.info("Recognizing your speech.......")

            try:
                text = self.init_rec.recognize_google(audio_data)
                self.logger.info(text)

            except sr.UnknownValueError:
                pass

        text = 'Bring me a bottle from dining'
        self.logger.info(f'Recognized text "{text}"')

        return text

    def select_answer(self, question):
        self.logger.info(f'Get text "{question}"')

        answer = ''
        ratio = 0.0
        t_ratio = 0.0

        for key, value in self.Questions.items():
            t_ratio = Levenshtein.ratio(question, key)
            self.logger.info(f'Compared "{key}" and "{question}"')
            self.logger.info(f'Ratio : {round(t_ratio, 3)}')

            if t_ratio > ratio:
                ratio = t_ratio
                answer = value


        return answer

    def speech_synthesis(self, text):
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

    speech_of_bring_me = SpeechOfBringMe()

    rclpy.spin(speech_of_bring_me)
    speech_of_bring_me.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
