import rclpy
import rclpy.node
from std_msgs.msg import String

from gtts import gTTS
from subprocess import run, PIPE

import os

class SpeechSynthesis(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_synthesis")    
    
        self.CMD = "mpg321 voice.mp3"    
    
        self.logger = self.get_logger()        
        self.logger.info("Start selection answer")    
    
        self.answer_sub = self.create_subscription(String, "answer_text", self.answer_subscribe, 10)    
            
    def answer_subscribe(self, msg):    
        self.logger.info("Subscribe text '{}'".format(msg.data))    
        tts = gTTS(msg.data)    
        tts.save("voice.mp3")    
            
        run(self.CMD.split(), stdout=PIPE, stderr=PIPE)    
        self.logger.info("Speeched text")    
        os.remove("voice.mp3")    
        
def main():            
    rclpy.init()            
            
    speech_synthesis = SpeechSynthesis()    
        
    try:    
        rclpy.spin(speech_synthesis)    
    except:    
        speech_synthesis.destroy_node()    
    
    rclpy.shutdown()
