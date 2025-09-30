#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        
        # Create publisher to send shape commands
        self.shape_publisher = self.create_publisher(String, 'shape_command', 10)
        
        self.get_logger().info('ShapeNode has been started')
        
        # Start user input in separate thread
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()
        
        # Timer to keep the node alive
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        # This keeps the node alive
        pass
        
    def user_input_loop(self):
        """Handle user input in a separate thread"""
        while rclpy.ok():
            try:
                self.display_menu()
                choice = input("Enter your choice (1-4): ").strip()
                
                if choice == '1':
                    self.send_shape_command('spiral')
                elif choice == '2':
                    self.send_shape_command('heart')
                elif choice == '3':
                    self.send_shape_command('rose')
                elif choice == '4':
                    self.send_shape_command('stop')
                    self.get_logger().info('Stopping turtle...')
                    break
                else:
                    print("Invalid choice! Please select 1-4.")
                    
                time.sleep(0.1)  # Small delay to prevent busy waiting
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
                
    def display_menu(self):
        """Display the shape selection menu"""
        print("\n" + "="*50)
        print("🐢 TURTLE SHAPE DRAWER 🐢")
        print("="*50)
        print("Select a shape to draw:")
        print("1. 🌀 Spiral (Archimedean)")
        print("2. ❤️  Heart (Cardioid)")
        print("3. 🌸 Rose (Flower)")
        print("4. 🛑 Stop Turtle")
        print("="*50)
        
    def send_shape_command(self, shape):
        """Send shape command to turtle commander"""
        msg = String()
        msg.data = shape
        self.shape_publisher.publish(msg)
        
        if shape != 'stop':
            self.get_logger().info(f'Sending command to draw: {shape}')
        else:
            self.get_logger().info('Sending stop command')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        shape_node = ShapeNode()
        rclpy.spin(shape_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()