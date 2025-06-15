import rclpy
from rclpy.node import Node
from custom_interfaces.msg import HelloMessage

class PublisherNode(Node):
    
    def __init__(self):
        super().__init__('Publisher_node') #Default Node Name
        
        # Declaring and Getting Parameter
        self.declare_parameter("topic", value='hello_message')
        topic_name = self.get_parameter("topic").get_parameter_value().string_value
        
        #Creating a Publisher
        self.publisher_ = self.create_publisher(HelloMessage, topic_name, 10)
        self.time_period = 0.5
        self.count = 0
        
        # Initiating the Timer Callback - to publish messages at every 0.5 seconds.
        self.timer = self.create_timer(self.time_period, self.timer_callback)
    
    def timer_callback(self):
        msg = HelloMessage()
        msg.message = f"Hello Everyone the message id - {self.count}"
        self.publisher_.publish(msg)
        self.count += 1
        self.get_logger().info(f"Publishing {msg.message}")


def main(args=None):
    #Setup RCLPY Context
    rclpy.init(args=args)
    
    # Create Node
    pub_node = PublisherNode()
    
    #Spin Node
    rclpy.spin(pub_node)
    
    # Destroy Node
    pub_node.destroy_node()
    
    # Destroy Context
    rclpy.shutdown()

if __name__ == main():
    main()