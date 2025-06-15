import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import Fibonacci

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server_node')
        
        # Declaring and Getting Parameter
        self.declare_parameter("action_service", value='fibonacci_service')
        action_service_name = self.get_parameter("action_service").get_parameter_value().string_value
        
        #Create Action Server
        self._action_server = ActionServer(self, Fibonacci, action_service_name, self.execute_callback)
        
        self.get_logger().info('Fibonacci Action Server started.')
    
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Goal.....')
        
        #Setting Up The Feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0,1]
        
        #Getting the Goal
        order = goal_handle.request.order # This call back gets Goal handle not Goal Class. Goal class resides with data resides in goal_handle.request. 
        # Goal Handle has many functions to handle the progress towards the goal like abort, succeed, canceled, and publish_feedback
        
        if order <= 0:
            goal_handle.abort()
            result = Fibonacci.Result()
            result.sequence = []
            return result
        
        # Navigating towards the Goal
        for i in range(1, order-1):
            next_value = feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            feedback_msg.partial_sequence.append(next_value)
            
            self.get_logger().info(f"Feedback: {feedback_msg.partial_sequence}")
            ## Send Feedback
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    #Setup RCLPY Context
    rclpy.init(args=args)
    
    # Create Node
    act_server_node = ActionServerNode()
    
    #Spin Node
    rclpy.spin(act_server_node)
    
    # Destroy Node
    act_server_node.destroy_node()
    
    # Destroy Context
    rclpy.shutdown()

if __name__ == '__main__':
    main()