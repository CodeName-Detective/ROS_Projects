import rclpy
from rclpy.node import Node
from custom_interfaces.msg import HelloMessage


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node') #Default Node Name
        
        # Declaring and Getting Parameter
        self.declare_parameter("topic", value='hello_message')
        topic_name = self.get_parameter("topic").get_parameter_value().string_value
        
        #Create Subscription Callback
        self.subscription_ = self.create_subscription(HelloMessage, topic_name, self.listener_callback, 10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f"Received {msg.message}")


def main(args=None):
    #Setup RCLPY Context
    rclpy.init(args=args)
    
    # Create Node
    sub_node = SubscriberNode()
    
    #Spin Node
    rclpy.spin(sub_node)
    
    # Destroy Node
    sub_node.destroy_node()
    
    # Destroy Context
    rclpy.shutdown()

if __name__ == main():
    main()