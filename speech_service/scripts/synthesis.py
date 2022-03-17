import rclpy
import rclpy.node
from ai_robot_book_interfaces.srv import StringCommand

from gtts import gTTS
from io import BytesIO
from mpg123 import Mpg123, Out123


class SpeechSynthesis(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_synthesis")    
    
        self.logger = self.get_logger()        
        self.logger.info("音声合成ノードを開始します")

        self.lang = 'ja-JP'    
        self.mp3 = Mpg123()    
        # self.out = Out123()
    
        self.service = self.create_service(StringCommand, '/speech/synthesis', self.synthesis)
            
    def synthesis(self, request, response):
        self.get_logger().info('音声合成')

        if not len(request.command):
            response.answer = 'failed'
            return response

        tts = gTTS(request.command, lang=self.lang[:2])    
        fp = BytesIO()    
        tts.write_to_fp(fp)    
        fp.seek(0)    
        self.mp3.feed(fp.read())    
        for frame in self.mp3.iter_frames(self.out.start):    
            self.out.play(frame)

        response.answer = 'succeeded'
        return response
        
def main():            
    rclpy.init()            
            
    speech_synthesis = SpeechSynthesis()    
        
    try:    
        rclpy.spin(speech_synthesis)    
    except:    
        speech_synthesis.destroy_node()    
    
    rclpy.shutdown()
