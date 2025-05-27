#!/usr/bin/env python3

"""
A simple ROS 2 node demonstrating parameter usage.
This example shows how to:
- Declare parameters of different types
- Get parameter values
- Set up parameter change callbacks
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult


class ParametersDemo(Node):
    """Node demonstrating ROS 2 parameters."""

    def __init__(self):
        """Initialize the parameters demo node."""
        super().__init__('parameters_demo')
        
        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'int_param', 
            42,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='An integer parameter example'
            )
        )
        
        self.declare_parameter(
            'double_param', 
            3.14,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='A floating point parameter example'
            )
        )
        
        self.declare_parameter(
            'string_param', 
            'Hello ROS 2',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='A string parameter example'
            )
        )
        
        self.declare_parameter(
            'bool_param', 
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='A boolean parameter example'
            )
        )
        
        self.declare_parameter(
            'array_param', 
            [1, 2, 3, 4],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description='An integer array parameter example'
            )
        )
        
        # Set up a timer to periodically print parameter values
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Add a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info('Parameters demo node initialized')
        self.print_current_parameters()

    def timer_callback(self):
        """Timer callback to periodically print parameter values."""
        self.print_current_parameters()

    def print_current_parameters(self):
        """Print all current parameter values."""
        int_param = self.get_parameter('int_param').value
        double_param = self.get_parameter('double_param').value
        string_param = self.get_parameter('string_param').value
        bool_param = self.get_parameter('bool_param').value
        array_param = self.get_parameter('array_param').value
        
        self.get_logger().info(f'Current parameters:')
        self.get_logger().info(f'  int_param: {int_param}')
        self.get_logger().info(f'  double_param: {double_param}')
        self.get_logger().info(f'  string_param: {string_param}')
        self.get_logger().info(f'  bool_param: {bool_param}')
        self.get_logger().info(f'  array_param: {array_param}')

    def parameters_callback(self, params):
        """
        Callback for parameter changes.
        
        This is called whenever a parameter is changed. You can validate
        parameter changes here and reject invalid values.
        """
        for param in params:
            self.get_logger().info(f'Parameter changed: {param.name} = {param.value}')
            
            # Example of parameter validation
            if param.name == 'int_param' and param.value < 0:
                self.get_logger().warn('int_param cannot be negative, rejecting change')
                return SetParametersResult(successful=False, reason='int_param cannot be negative')
                
        return SetParametersResult(successful=True)


def main(args=None):
    """Run the parameters demo node."""
    rclpy.init(args=args)
    node = ParametersDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
