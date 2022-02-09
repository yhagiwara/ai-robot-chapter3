import rclpy    
import rclpy.node    
from std_msgs.msg import String    
    
import Levenshtein    
    
    
class SelectAnswer(rclpy.node.Node):        
    def __init__(self):        
        super().__init__("select_answer")    
        
        self.logger = self.get_logger()    
        self.logger.info("Start selection answer")    
    
        self.Questions = {"Bring me a bottle from kitchen":"Ok, I will"}    
    
        self.text_sub = self.create_subscription(String, "recognized_text", self.select_answer, 10)    
        self.answer_pub = self.create_publisher(String, "answer_text", 10)    
        
    def select_answer(self, msg):    
        self.logger.info("Subscribe text '{}'".format(msg.data))    
    
        answer = ""    
        ratio = 0.0    
        t_ratio = 0.0    
    
        for key, value in self.Questions.items():    
            t_ratio = Levenshtein.ratio(msg.data, key)    
            self.logger.info("Ratio : {} | Compared '{}' and '{}'".format(round(t_ratio, 3), key, msg.data))    
    
            if t_ratio > ratio:    
                ratio = t_ratio    
                answer = value    
    
        if ratio > 0.8:    
            msg = String()    
            msg.data = answer    
            self.answer_pub.publish(msg)    
            self.logger.info("Published answer text '{}'".format(msg.data))    
        
        
def main():        
    rclpy.init()        
        
    select_answer = SelectAnswer()    
    
    try:    
        rclpy.spin(select_answer)    
    except:    
        select_answer.destroy_node()    
    
    rclpy.shutdown()
