import rclpy
from rclpy.node import Node
from custom_interfaces.srv import AddTwoInts

class ServerNode(Node):
    
    def __init__(self):
        super().__init__('server_node') #Default Node Name
        
        # Declaring and Getting Parameter
        self.declare_parameter("service", value='add_two_ints')
        service_name = self.get_parameter("service").get_parameter_value().string_value
        
        #Create Service
        self.service_ = self.create_service(AddTwoInts, service_name, self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum  = request.a + request.b
        self.get_logger().info(f"Incoming Request a: {request.a}, b:{request.b}")
        return response


def main(args=None):
    #Setup RCLPY Context
    rclpy.init(args=args)
    
    # Create Node
    serv_node = ServerNode()
    
    #Spin Node
    rclpy.spin(serv_node)
    
    # Destroy Node
    serv_node.destroy_node()
    
    # Destroy Context
    rclpy.shutdown()

if __name__ == '__main__':
    main()