#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
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
        
        # Service clients - only create if needed
        self.set_pen_client = None
        self.clear_client = None
        
        # Turtle state
        self.current_pose = None
        self.is_drawing = False
        self.current_shape = None
        
        # Shape parameters
        self.scale_factor = 1.5
        self.drawing_speed = 2.0
        self.angular_speed = 2.0
        
        self.get_logger().info('TurtleCommander has been started')
        
    def pose_callback(self, msg):
        """Update current turtle pose"""
        self.current_pose = msg
        
    def shape_callback(self, msg):
        """Handle incoming shape commands"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'stop':
            self.stop_turtle()
        elif command == 'clear':
            self.clear_screen()
        elif command in ['spiral', 'heart', 'star']:
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
        
    def clear_screen(self):
        """Clear the turtle screen and reset turtle position"""
        try:
            # Create client only when needed
            if self.clear_client is None:
                self.clear_client = self.create_client(Empty, '/clear')
            
            if self.clear_client.wait_for_service(timeout_sec=2.0):
                request = Empty.Request()
                future = self.clear_client.call_async(request)
                self.get_logger().info('Screen cleared')
                
                # Wait a bit for the clear to complete, then reset turtle position
                time.sleep(0.5)
                self.reset_turtle_position()
            else:
                self.get_logger().warn('Clear service not available')
        except Exception as e:
            self.get_logger().error(f'Failed to clear screen: {e}')
    
    def reset_turtle_position(self):
        """Reset turtle to center position"""
        # Lift pen
        self.set_pen_state(off=True)
        time.sleep(0.2)
        
        # Move to center
        self.move_to_center_direct()
        
        # Put pen down
        self.set_pen_state(off=False)
        self.get_logger().info('Turtle position reset to center')
        
    def set_pen_color(self, r, g, b):
        """Set pen color"""
        try:
            # Create client only when needed
            if self.set_pen_client is None:
                self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
                
            if self.set_pen_client.wait_for_service(timeout_sec=2.0):
                request = SetPen.Request()
                request.r = r
                request.g = g
                request.b = b
                request.width = 3
                request.off = 0  # Pen down for drawing
                future = self.set_pen_client.call_async(request)
            else:
                self.get_logger().warn('SetPen service not available')
        except Exception as e:
            self.get_logger().error(f'Failed to set pen: {e}')
    
    def set_pen_state(self, off=False):
        """Set pen up or down"""
        try:
            if self.set_pen_client is None:
                self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
                
            if self.set_pen_client.wait_for_service(timeout_sec=2.0):
                request = SetPen.Request()
                request.r = 255  # Keep current color
                request.g = 255
                request.b = 255
                request.width = 3
                request.off = 1 if off else 0  # 1 = pen up, 0 = pen down
                future = self.set_pen_client.call_async(request)
            else:
                self.get_logger().warn('SetPen service not available')
        except Exception as e:
            self.get_logger().error(f'Failed to set pen state: {e}')
        
    def draw_shape(self, shape):
        """Draw the specified shape"""
        if self.current_pose is None:
            self.get_logger().warn('Turtle pose not available yet')
            return
            
        self.is_drawing = True
        self.get_logger().info(f'Starting to draw {shape}')
        
        # Lift pen and move to starting position
        self.set_pen_state(off=True)
        time.sleep(0.3)
        
        # Set pen color based on shape and put pen down
        if shape == 'spiral':
            self.set_pen_color(255, 0, 0)  # Red
        elif shape == 'heart':
            self.set_pen_color(255, 20, 147)  # Pink
        elif shape == 'star':
            self.set_pen_color(255, 215, 0)  # Gold
        
        time.sleep(0.3)  # Wait for pen color to set
        
        try:
            if shape == 'spiral':
                self.draw_spiral()
            elif shape == 'heart':
                self.draw_heart()
            elif shape == 'star':
                self.draw_star()
        except Exception as e:
            self.get_logger().error(f'Error drawing shape: {e}')
        finally:
            self.is_drawing = False
            self.stop_turtle()
            self.get_logger().info(f'Finished drawing {shape}')
    
    def draw_spiral(self):
        """Draw Archimedean spiral"""
        self.get_logger().info('Drawing Archimedean Spiral')
       
        # Move to center with pen up
        self.move_to_center_direct()
       
        # Put pen down to start drawing
        self.set_pen_state(off=False)
        time.sleep(0.3)
       
        # Draw larger spiral with continuous motion
        total_time = 15.0
        steps = 500
        dt = total_time / steps
       
        for i in range(steps):
            if not self.is_drawing:
                break
               
            # Calculate spiral parameters
            progress = i / steps
           
            # Archimedean spiral: constant spacing between turns
            # Linear velocity increases smoothly for larger spiral
            linear_vel = 0.3 + progress * 3.5  # Increased for larger spiral
            angular_vel = 1.5  # Constant rotation speed for even spacing
           
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
           
            self.cmd_vel_publisher.publish(twist)
            time.sleep(dt)
    
    def draw_heart(self):
        """Draw symmetric heart shape"""
        self.get_logger().info('Drawing Heart')
        
        # Move to center first
        self.move_to_center_direct()
        time.sleep(0.5)
        
        # Put pen down at center
        self.set_pen_state(off=False)
        time.sleep(0.5)
        
        self.get_logger().info('Starting to draw heart shape')
        
        # Draw heart using velocity commands
        total_time = 20.0
        steps = 400
        dt = total_time / steps
        
        for i in range(steps + 1):
            if not self.is_drawing:
                break
            
            # Parameter t goes from 0 to 2π
            t = (i / steps) * 2 * math.pi
            
            # Heart parametric equations
            sin_t = math.sin(t)
            cos_t = math.cos(t)
            
            # Position (NOT flipped, keep original orientation)
            x = 16 * (sin_t ** 3)
            y = 13 * cos_t - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
            
            # Scale and center
            scale = 0.20
            x = x * scale + 5.5
            y = y * scale + 5.5  # No flip
            
            # Calculate derivatives for velocity
            dx_dt = 48 * (sin_t ** 2) * cos_t
            dy_dt = -13 * sin_t + 10 * math.sin(2*t) + 6 * math.sin(3*t) + 4 * math.sin(4*t)
            
            # Scale derivatives
            dx_dt *= scale
            dy_dt *= scale
            
            # Calculate speed
            speed = math.sqrt(dx_dt**2 + dy_dt**2)
            
            # Normalize speed for turtle movement
            linear_vel = speed * 0.10
            linear_vel = max(0.5, min(linear_vel, 2.5))
            
            # Calculate required heading angle
            if abs(dx_dt) > 0.001 or abs(dy_dt) > 0.001:
                vel_angle = math.atan2(dy_dt, dx_dt)
                
                # Calculate angular velocity needed
                if self.current_pose:
                    angle_diff = vel_angle - self.current_pose.theta
                    
                    # Normalize angle
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Smooth angular velocity
                    angular_vel = angle_diff / dt * 0.7
                    angular_vel = max(-4.0, min(angular_vel, 4.0))
                else:
                    angular_vel = 0.0
            else:
                angular_vel = 0.0
            
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            
            self.cmd_vel_publisher.publish(twist)
            time.sleep(dt)
        
        self.get_logger().info('Heart drawing complete')
    
    def draw_star(self):
        """Draw 5-pointed star using parametric approach"""
        self.get_logger().info('Drawing 5-pointed Star')
        
        # Use a parametric star shape based on radius modulation
        # r(θ) varies between inner and outer radius creating star points
        
        # Move to center
        self.move_to_center_direct()
        time.sleep(0.5)
        
        # Put pen down
        self.set_pen_state(off=False)
        time.sleep(0.5)
        
        self.get_logger().info('Starting to draw star')
        
        # Star parameters
        outer_radius = 2.5
        inner_radius = 1.0
        num_points = 5
        
        # We'll create a smooth transition between inner and outer radius
        # as we go around the circle
        total_time = 15.0
        steps = 500
        dt = total_time / steps
        
        for i in range(steps + 1):
            if not self.is_drawing:
                break
            
            # Angle around the circle
            theta = (i / steps) * 2 * math.pi
            
            # Modulate radius to create star points
            # Use a sawtooth wave that goes: outer -> inner -> outer -> inner ...
            # There are 5 points, so 10 transitions in full circle
            phase = (theta * num_points) % (2 * math.pi)
            
            # Create sharp points using a triangle wave
            if phase < math.pi:
                # Going from outer to inner
                t = phase / math.pi
                r = outer_radius - (outer_radius - inner_radius) * t
            else:
                # Going from inner to outer
                t = (phase - math.pi) / math.pi
                r = inner_radius + (outer_radius - inner_radius) * t
            
            # Calculate position
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            
            # Calculate velocity
            # dr/dtheta - derivative of radius with respect to angle
            if phase < math.pi:
                dr_dtheta = -(outer_radius - inner_radius) * num_points / math.pi
            else:
                dr_dtheta = (outer_radius - inner_radius) * num_points / math.pi
            
            # Velocity in Cartesian coordinates
            dx_dtheta = dr_dtheta * math.cos(theta) - r * math.sin(theta)
            dy_dtheta = dr_dtheta * math.sin(theta) + r * math.cos(theta)
            
            speed = math.sqrt(dx_dtheta**2 + dy_dtheta**2)
            
            # Convert to linear velocity
            linear_vel = speed * 0.08
            linear_vel = max(0.5, min(linear_vel, 2.5))
            
            # Calculate heading
            if abs(dx_dtheta) > 0.001 or abs(dy_dtheta) > 0.001:
                vel_angle = math.atan2(dy_dtheta, dx_dtheta)
                
                if self.current_pose:
                    angle_diff = vel_angle - self.current_pose.theta
                    
                    # Normalize
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    angular_vel = angle_diff / dt * 0.8
                    angular_vel = max(-5.0, min(angular_vel, 5.0))
                else:
                    angular_vel = 0.0
            else:
                angular_vel = 0.0
            
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            
            self.cmd_vel_publisher.publish(twist)
            time.sleep(dt)
        
        self.get_logger().info('Star drawing complete')
    
    def move_forward(self, distance):
        """Move forward by a specific distance"""
        if self.current_pose is None:
            return
        
        # Calculate time based on speed
        speed = 2.0
        move_time = distance / speed
        
        # Move forward
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_publisher.publish(twist)
        time.sleep(move_time)
        
        # Stop
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    def rotate_in_place(self, angle):
        """Rotate by a specific angle (positive=counterclockwise, negative=clockwise)"""
        if self.current_pose is None:
            return
        
        # Calculate time based on angular speed
        angular_speed = 2.0  # radians per second
        turn_time = abs(angle) / angular_speed
        
        # Rotate
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -angular_speed
        self.cmd_vel_publisher.publish(twist)
        time.sleep(turn_time)
        
        # Stop
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    def move_straight_to(self, target_x, target_y):
        """Move in a straight line to target"""
        if self.current_pose is None:
            return
        
        # First, turn to face the target
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.05:
            return
        
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        
        # Turn to face target
        angle_diff = target_angle - self.current_pose.theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Turn in place first
        if abs(angle_diff) > 0.1:
            turn_time = abs(angle_diff) / 2.0
            twist = Twist()
            twist.angular.z = 2.0 if angle_diff > 0 else -2.0
            self.cmd_vel_publisher.publish(twist)
            time.sleep(turn_time)
            
            # Stop turning
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        
        # Now move straight forward
        move_time = distance / 2.0  # Speed of 2.0 units/sec
        twist = Twist()
        twist.linear.x = 2.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(move_time)
        
        # Stop
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    def move_to_center(self):
        """Move turtle to center"""
        if self.current_pose is None:
            return
            
        center_x, center_y = 5.5, 5.5
        self.move_to_point(center_x, center_y)
    
    def move_to_center_direct(self):
        """Move turtle to center without drawing"""
        if self.current_pose is None:
            return
            
        center_x, center_y = 5.5, 5.5
        self.move_to_point_pen_up(center_x, center_y)
    
    def move_to_point_pen_up(self, target_x, target_y):
        """Move to point with pen lifted (no drawing)"""
        if self.current_pose is None:
            return
        
        # First turn towards the target
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.05:
            return
        
        # Calculate target angle and turn
        target_angle = math.atan2(dy, dx)
        self.turn_to_angle(target_angle)
        
        # Move straight to target
        move_time = distance / 2.0  # Speed of 2.0 units/sec
        twist = Twist()
        twist.linear.x = 2.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(move_time)
        
        # Stop
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    def turn_to_angle(self, target_angle):
        """Turn turtle to face target angle"""
        if self.current_pose is None:
            return
            
        angle_diff = target_angle - self.current_pose.theta
        
        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) < 0.05:  # Already facing the right direction
            return
            
        # Turn to face target
        turn_time = abs(angle_diff) / 3.0  # Angular speed of 3.0 rad/sec
        twist = Twist()
        twist.angular.z = 3.0 if angle_diff > 0 else -3.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(turn_time)
        
        # Stop turning
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    def turn_and_move_to(self, target_x, target_y):
        """Turn and move to target position"""
        if self.current_pose is None:
            return
        
        # Calculate angle to target
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            return
        
        target_angle = math.atan2(dy, dx)
        
        # Turn to face target
        self.turn_to_angle(target_angle)
        
        # Move forward
        move_time = distance / 2.0
        twist = Twist()
        twist.linear.x = 2.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(move_time)
        
        # Stop
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
    
    def move_to_point(self, target_x, target_y):
        """Move turtle to a specific point"""
        if self.current_pose is None:
            return
            
        # Calculate distance and angle
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.05:
            return
            
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pose.theta
        
        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create movement command
        twist = Twist()
        
        # If we need to turn significantly, turn first
        if abs(angle_diff) > 0.2:
            twist.angular.z = 2.0 * angle_diff
            twist.linear.x = 0.5  # Small forward movement while turning
        else:
            # Move forward with minor angle correction
            twist.linear.x = min(2.0, distance * 3.0)
            twist.angular.z = angle_diff * 2.0
        
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