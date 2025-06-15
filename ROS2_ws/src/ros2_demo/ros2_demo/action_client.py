import os
import sys
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces.action import Fibonacci
from ament_index_python.packages import get_package_share_directory



class ActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node')
        
        
        # Declaring and Getting Parameter
        package_share_dir = get_package_share_directory('ros2_demo')
        yaml_file = os.path.join(package_share_dir, 'config', 'parameters.yaml')
        with open(yaml_file, 'r') as f:
            parameters = yaml.safe_load(f)
        
        # Extract your node parameters section
        node_parameters = parameters.get('action_client_node', {}).get('ros__parameters', {})
        
        # Declare and set parameters
        for key, value in node_parameters.items():
            self.declare_parameter(key, value)
        
        action_service_name = self.get_parameter('action_service').value
        
        #Create ACtion Client
        self._action_client = ActionClient(self, Fibonacci, action_service_name)
        
        #Wait for the server to be ready.
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Action Service not available, waiting again.....")
        
        
        self.get_logger().info('Fibonacci action client started')
    
    
    def send_goal(self, order):
        
        #pack the Goal Data
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        #send Goal Asynchronously with a Feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback = self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        
        #Collect the Goal Response Callback- Called when server replies whether the goal is accepted or rejected.
        self._send_goal_future.add_done_callback(self.goal_response_callback) #It registers a callback function that will be called once the future is complete — i.e., once the asynchronous task has finished.
        
    
    def feedback_callback(self, feedback_msg):
        #Called When Feedback is received from the server.
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')
    
    
    def goal_response_callback(self, future):
        #Called when server replies whether the goal is accepted or rejected.
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        
        #Get the result Asynchronously
        self._get_result_future = goal_handle.get_result_async()
        
        # Get and Process the final result
        self._get_result_future.add_done_callback(self.get_result_callback) #It registers a callback function that will be called once the future is complete — i.e., once the asynchronous task has finished.
    
    
    def get_result_callback(self, future):
        #process the result fo the goal
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()



def main(args=None):
    #Setup RCLPY Context
    rclpy.init(args=args)
    
    order = int(sys.argv[1])
    
    # Create Node
    act_client_node = ActionClientNode()
    act_client_node.send_goal(order) #Send the goal async
    
    #Spin Node
    rclpy.spin(act_client_node)
    
    # Destroy Node
    #act_client_node.destroy_node()
    
    # Destroy Context
    #rclpy.shutdown()

if __name__ == '__main__':
    main()