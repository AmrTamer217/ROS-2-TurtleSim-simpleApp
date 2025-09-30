#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
import math
import threading
import time

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        
        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.shape_subscriber = self.create_subscription(
            String, 'shape_command', self.shape_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )
        
        # Turtle state
        self.current_pose = None
        self.is_drawing = False
        self.current_shape = None
        
        # Shape parameters
        self.scale_factor = 2.0
        self.drawing_speed = 1.0
        
        self.get_logger().info('TurtleCommander has been started')
        
    def pose_callback(self, msg):
        """Update current turtle pose"""
        self.current_pose = msg
        
    def shape_callback(self, msg):
        """Handle incoming shape commands"""
        command = msg.data.lower()
        
        if command == 'stop':
            self.stop_turtle()
        elif command in ['spiral', 'heart', 'rose']:
            if not self.is_drawing:
                self.current_shape = command
                threading.Thread(target=self.draw_shape, args=(command,), daemon=True).start()
            else:
                self.get_logger().warn('Already drawing a shape. Please wait...')
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            
    def stop_turtle(self):
        """Stop the turtle movement"""
        self.is_drawing = False
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Turtle stopped')
        
    def draw_shape(self, shape):
        """Draw the specified shape"""
        if self.current_pose is None:
            self.get_logger().warn('Turtle pose not available yet')
            return
            
        self.is_drawing = True
        self.get_logger().info(f'Starting to draw {shape}')
        
        try:
            if shape == 'spiral':
                self.draw_spiral()
            elif shape == 'heart':
                self.draw_heart()
            elif shape == 'rose':
                self.draw_rose()
        except Exception as e:
            self.get_logger().error(f'Error drawing shape: {e}')
        finally:
            self.is_drawing = False
            self.stop_turtle()
            self.get_logger().info(f'Finished drawing {shape}')
    
    def draw_spiral(self):
        """Draw Archimedean spiral: r = a + b*theta"""
        self.get_logger().info('Drawing Archimedean Spiral')
        
        # Parameters
        a = 0.2  # Initial radius
        b = 0.3  # Growth rate
        max_theta = 6 * math.pi  # 3 full rotations
        steps = 200
        
        # Calculate points
        points = []
        for i in range(steps):
            if not self.is_drawing:
                break
                
            theta = (i / steps) * max_theta
            r = a + b * theta
            
            # Convert to Cartesian coordinates
            x = r * math.cos(theta) * self.scale_factor
            y = r * math.sin(theta) * self.scale_factor
            
            # Center around turtle's starting position
            center_x = 5.5
            center_y = 5.5
            
            points.append((center_x + x, center_y + y))
        
        self.move_through_points(points)
    
    def draw_heart(self):
        """Draw heart shape (cardioid): r = a(1 + cos(theta))"""
        self.get_logger().info('Drawing Heart (Cardioid)')
        
        # Parameters
        a = 1.5  # Size parameter
        steps = 100
        
        # Calculate points
        points = []
        for i in range(steps + 1):  # +1 to close the shape
            if not self.is_drawing:
                break
                
            theta = (i / steps) * 2 * math.pi
            r = a * (1 + math.cos(theta))
            
            # Convert to Cartesian coordinates
            x = r * math.cos(theta) * self.scale_factor
            y = r * math.sin(theta) * self.scale_factor
            
            # Center and orient the heart
            center_x = 5.5
            center_y = 5.5
            
            points.append((center_x + x, center_y + y))
        
        self.move_through_points(points)
    
    def draw_rose(self):
        """Draw rose pattern: r = a*cos(n*theta)"""
        self.get_logger().info('Drawing Rose (Flower)')
        
        # Parameters
        a = 2.0   # Size parameter
        n = 4     # Number of petals (even number gives n petals)
        steps = 200
        
        # Calculate points
        points = []
        for i in range(steps + 1):
            if not self.is_drawing:
                break
                
            theta = (i / steps) * 2 * math.pi
            r = abs(a * math.cos(n * theta))  # abs to handle negative values
            
            # Convert to Cartesian coordinates
            x = r * math.cos(theta) * self.scale_factor
            y = r * math.sin(theta) * self.scale_factor
            
            # Center the rose
            center_x = 5.5
            center_y = 5.5
            
            points.append((center_x + x, center_y + y))
        
        self.move_through_points(points)
    
    def move_through_points(self, points):
        """Move turtle through a series of points"""
        if not points or self.current_pose is None:
            return
            
        for target_x, target_y in points:
            if not self.is_drawing:
                break
                
            self.move_to_point(target_x, target_y)
            time.sleep(0.05)  # Small delay between points
    
    def move_to_point(self, target_x, target_y):
        """Move turtle to a specific point"""
        if self.current_pose is None:
            return
            
        # Calculate distance and angle to target
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Close enough
            return
            
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pose.theta
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create velocity command
        twist = Twist()
        
        # If angle difference is large, turn first
        if abs(angle_diff) > 0.1:
            twist.angular.z = 3.0 * angle_diff
        else:
            # Move forward with slight angular correction
            twist.linear.x = min(self.drawing_speed, distance * 2.0)
            twist.angular.z = 2.0 * angle_diff
        
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        turtle_commander = TurtleCommander()
        rclpy.spin(turtle_commander)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()