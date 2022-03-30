import rclpy
import rclpy.node
from std_msgs.msg import String

from gtts import gTTS
from io import BytesIO
from mpg123 import Mpg123, Out123


class Synthesis(rclpy.node.Node):
    def __init__(self):
        super().__init__("speech_synthesis")

        self.get_logger().info('音声合成ノードを起動します')

        self.lang = 'ja-JP'
        self.mp3 = Mpg123()
        self.out = Out123()
    
        self.subscriber = self.create_subscription(String, '/speech', self.synthesis, 1)

    def synthesis(self, msg):
        self.get_logger().info('音声合成を実行します')

        text = msg.data
        self.get_logger().info(f'\"{text}\"と発話します')

        tts = gTTS(text, lang=self.lang[:2])
        fp = BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)
        self.mp3.feed(fp.read())

        for frame in self.mp3.iter_frames(self.out.start):
            self.out.play(frame)

        
def main():
    rclpy.init()

    synthesis_node = Synthesis()

    try:
        rclpy.spin(synthesis_node)
    except:
        synthesis_node.destroy_node()

    rclpy.shutdown()
