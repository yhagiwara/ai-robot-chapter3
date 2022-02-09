import rclpy    
import rclpy.node    
from std_msgs.msg import String    
    
from time import sleep    
import speech_recognition as sr    
import pyaudio    
    
class SpeechRecognition(rclpy.node.Node):    
    def __init__(self):    
        super().__init__("speech_recognition")    
    
        self.logger = self.get_logger()    
        self.logger.info("Start speech recognition")    
    
        self.period = 5.0    
        self.init_rec = sr.Recognizer()    
    
        self.recognized_pub = self.create_publisher(String, "recognized_text", 10)    
    
        self.timer = self.create_timer(self.period, self.recognition)    
    
    def recognition(self):    
        msg = String()    
    
        with sr.Microphone() as source:    
            audio_data = self.init_rec.record(source, duration=5)    
            self.logger.info("Recognizing your speech.......")    
    
            try:    
                text = self.init_rec.recognize_google(audio_data)    
                self.logger.info(text)    
                msg.data = text    
    
            except sr.UnknownValueError:    
                pass    
    
        #msg.data = "Bring me a bottle from dining"    
        self.recognized_pub.publish(msg)    
        self.logger.info("Published recognized text '{}'".format(msg.data))    
    
    
def main():    
    rclpy.init()    
    
    speech_recognition = SpeechRecognition()    
    
    rclpy.spin(speech_recognition)    
    speech_recognition.destroy_node()    
    
    rclpy.shutdown()
