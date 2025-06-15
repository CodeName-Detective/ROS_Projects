import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import AddTwoInts
from ament_index_python.packages import get_package_share_directory


class ClientNodeAsync(Node):
    def __init__(self):
        super().__init__('client_node_async') #Default Node Name
        
        # Declaring and Getting Parameter
        package_share_dir = get_package_share_directory('ros2_demo')
        yaml_file = os.path.join(package_share_dir, 'config', 'parameters.yaml')
        with open(yaml_file, 'r') as f:
            parameters = yaml.safe_load(f)
        
        # Extract your node parameters section
        node_parameters = parameters.get('client_node_async', {}).get('ros__parameters', {})
        
        # Declare and set parameters
        for key, value in node_parameters.items():
            self.declare_parameter(key, value)
        
        service_name = self.get_parameter('service').value
        
        #Creating Client
        self.client_ = self.create_client(AddTwoInts, service_name)
        
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again.....")
        
        self.get_logger().info("Service Is available")
    
    def send_request(self):
        # Not a callback, have to be triggered by us.
        
        #Pack InputData
        request = AddTwoInts.Request()
        request.a = int(sys.argv[1])
        request.b = int(sys.argv[2])
        
        #Making an Asynchronous Request to the service
        self.future_ = self.client_.call_async(request)


def main(args=None):
    #Setup RCLPY Context
    rclpy.init(args=args)
    
    # Create Node
    client_node = ClientNodeAsync()
    client_node.send_request() #Not a callback
    
    while rclpy.ok():
        #Spin Node Once
        rclpy.spin_once(client_node)
        
        if client_node.future_.done():
            
            try:
                response = client_node.future_.result()
            except Exception as e:
                client_node.get_logger().info(f"Service call failed: {e}")
            else:
                client_node.get_logger().info(f"Result of the addition is: {response.sum}")
            break
    
    # Destroy Node
    client_node.destroy_node()
    
    # Destroy Context
    rclpy.shutdown()

if __name__ == '__main__':
    main()