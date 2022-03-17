import rclpy    
import rclpy.node    
from std_msgs.msg import String    
    
from time import sleep    
import speech_recognition as sr    
import pyaudio    
    
class SpeechRecognition(rclpy.node.Node):    
    def __init__(self):    
        super().__init__("speech_recognition")    
        """
        ロボット「オブジェクトの名前を言ってください」
        ユーザー「コップ」
        ロボット「コップですね．わかりました．」
        """
    
        self.logger = self.get_logger()    
        self.logger.info("Start speech recognition")    
    
        self.period = 5.0    
        self.init_rec = sr.Recognizer()    
    
        self.service = self.create_service(StringCommand, '/speech/command', self.recognition)
    
        self.timer = self.create_timer(self.period, self.recognition)    
    
    def recognition(self, request, response):    
        msg = String()    
    
        with sr.Microphone() as source:    
            audio_data = self.init_rec.record(source, duration=5)    
            self.logger.info(f'Recognizing your speech for 5 sec')
    
            try:    
                text = self.init_rec.recognize_google(audio_data)    
                self.logger.info(text)    
                response.answer = text    
    
            except sr.UnknownValueError:    
                pass    
    
        #msg.data = "Bring me a bottle from dining"    
        self.logger.info("Published recognized text '{}'".format(msg.data))    
        return response
    
    
def main():    
    rclpy.init()    
    
    speech_recognition = SpeechRecognition()    
    
    rclpy.spin(speech_recognition)    
    speech_recognition.destroy_node()    
    
    rclpy.shutdown()
