import rclpy    
import rclpy.node    
from ai_robot_book_interfaces.srv import StringCommand

import speech_recognition as sr    
import pyaudio    
    
class SpeechRecognition(rclpy.node.Node):    
    def __init__(self):    
        super().__init__("speech_recognition")    
    
        self.logger = self.get_logger()    
        self.logger.info("音声認識を起動します")
    
        self.init_rec = sr.Recognizer()    
    
        self.service = self.create_service(StringCommand, '/speech/recognition', self.recognition)
    
    def recognition(self, request, response):    
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
    
        #msg.data = "Bring me a bottle from dining"    
        self.logger.info(f'クライアントに認識したテキスト "{text}" を返します')
        return response
    
    
def main():    
    rclpy.init()    
    
    speech_recognition = SpeechRecognition()    
    
    rclpy.spin(speech_recognition)    
    speech_recognition.destroy_node()    
    
    rclpy.shutdown()
