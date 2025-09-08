#!/usr/bin/env python3
"""
Python-based robot dashboard with GUI for real-time monitoring
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import math
import time
import threading
try:
    import tkinter as tk
    from tkinter import ttk
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False
    print("GUI not available, running in console mode")


class RobotDashboard(Node):
    def __init__(self):
        super().__init__('robot_dashboard')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'robot_camera/image_raw', self.image_callback, 10)
        
        # Initialize data
        self.reset_data()
        
        # GUI setup
        if GUI_AVAILABLE:
            self.setup_gui()
            self.update_gui_timer()
        else:
            # Console mode timer
            self.timer = self.create_timer(1.0, self.print_status)
            
        self.get_logger().info('Robot Dashboard started')

    def reset_data(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_yaw = 0.0
        self.camera_fps = 0.0
        self.image_count = 0
        self.last_image_time = time.time()
        self.start_time = time.time()

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.orientation_yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        current_time = time.time()
        self.image_count += 1
        
        # Calculate FPS
        time_diff = current_time - self.last_image_time
        if time_diff > 0:
            self.camera_fps = 1.0 / time_diff
        self.last_image_time = current_time

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Robot Dashboard")
        self.root.geometry("600x500")
        self.root.configure(bg='#2c3e50')
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = tk.Label(main_frame, text="🤖 ROBOT DASHBOARD", 
                              font=("Arial", 18, "bold"), 
                              fg='#ecf0f1', bg='#2c3e50')
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Motion section
        motion_frame = tk.LabelFrame(main_frame, text="Motion Status", 
                                   font=("Arial", 12, "bold"),
                                   fg='#3498db', bg='#34495e', 
                                   padx=10, pady=10)
        motion_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Motion labels
        self.linear_label = tk.Label(motion_frame, text="Linear Velocity: 0.000 m/s", 
                                   font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.linear_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        
        self.angular_label = tk.Label(motion_frame, text="Angular Velocity: 0.000 rad/s", 
                                    font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.angular_label.grid(row=1, column=0, sticky=tk.W, pady=2)
        
        # Position section
        position_frame = tk.LabelFrame(main_frame, text="Position & Orientation", 
                                     font=("Arial", 12, "bold"),
                                     fg='#e67e22', bg='#34495e', 
                                     padx=10, pady=10)
        position_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.x_label = tk.Label(position_frame, text="X Position: 0.000 m", 
                              font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.x_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        
        self.y_label = tk.Label(position_frame, text="Y Position: 0.000 m", 
                              font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.y_label.grid(row=1, column=0, sticky=tk.W, pady=2)
        
        self.yaw_label = tk.Label(position_frame, text="Orientation: 0.0°", 
                                font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.yaw_label.grid(row=2, column=0, sticky=tk.W, pady=2)
        
        # Camera section
        camera_frame = tk.LabelFrame(main_frame, text="Camera Status", 
                                   font=("Arial", 12, "bold"),
                                   fg='#27ae60', bg='#34495e', 
                                   padx=10, pady=10)
        camera_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.fps_label = tk.Label(camera_frame, text="Camera FPS: 0.0", 
                                font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.fps_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        
        self.frame_count_label = tk.Label(camera_frame, text="Total Frames: 0", 
                                        font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.frame_count_label.grid(row=1, column=0, sticky=tk.W, pady=2)
        
        # Status section
        status_frame = tk.LabelFrame(main_frame, text="System Status", 
                                   font=("Arial", 12, "bold"),
                                   fg='#9b59b6', bg='#34495e', 
                                   padx=10, pady=10)
        status_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.runtime_label = tk.Label(status_frame, text="Runtime: 0s", 
                                    font=("Courier", 11), fg='#ecf0f1', bg='#34495e')
        self.runtime_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        
        self.status_label = tk.Label(status_frame, text="Status: 🟢 ACTIVE", 
                                   font=("Courier", 11), fg='#2ecc71', bg='#34495e')
        self.status_label.grid(row=1, column=0, sticky=tk.W, pady=2)

    def update_gui_timer(self):
        def update():
            while True:
                try:
                    self.root.after(100, self.update_gui)
                    time.sleep(0.1)
                except:
                    break
        
        update_thread = threading.Thread(target=update, daemon=True)
        update_thread.start()

    def update_gui(self):
        if not GUI_AVAILABLE:
            return
            
        try:
            # Update motion status
            self.linear_label.config(text=f"Linear Velocity: {self.linear_vel:.3f} m/s")
            self.angular_label.config(text=f"Angular Velocity: {self.angular_vel:.3f} rad/s")
            
            # Update position
            self.x_label.config(text=f"X Position: {self.position_x:.3f} m")
            self.y_label.config(text=f"Y Position: {self.position_y:.3f} m")
            self.yaw_label.config(text=f"Orientation: {math.degrees(self.orientation_yaw):.1f}°")
            
            # Update camera status
            self.fps_label.config(text=f"Camera FPS: {self.camera_fps:.1f}")
            self.frame_count_label.config(text=f"Total Frames: {self.image_count}")
            
            # Update system status
            runtime = int(time.time() - self.start_time)
            self.runtime_label.config(text=f"Runtime: {runtime}s")
            
            # Determine robot status
            if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
                status_text = "Status: 🟢 MOVING"
                status_color = '#2ecc71'
            else:
                status_text = "Status: 🟡 IDLE"
                status_color = '#f39c12'
                
            self.status_label.config(text=status_text, fg=status_color)
            
        except Exception as e:
            self.get_logger().error(f"GUI update error: {e}")

    def print_status(self):
        """Console mode status printing"""
        print("\n" + "="*50)
        print("          ROBOT DASHBOARD")
        print("="*50)
        print(f"Linear Velocity:  {self.linear_vel:.3f} m/s")
        print(f"Angular Velocity: {self.angular_vel:.3f} rad/s")
        print(f"X Position:       {self.position_x:.3f} m")
        print(f"Y Position:       {self.position_y:.3f} m")
        print(f"Orientation:      {math.degrees(self.orientation_yaw):.1f}°")
        print(f"Camera FPS:       {self.camera_fps:.1f}")
        print(f"Frame Count:      {self.image_count}")
        runtime = int(time.time() - self.start_time)
        print(f"Runtime:          {runtime}s")
        print("="*50)

    def run_gui(self):
        if GUI_AVAILABLE:
            self.root.mainloop()


def main():
    rclpy.init()
    
    dashboard = RobotDashboard()
    
    if GUI_AVAILABLE:
        # Run ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(dashboard), daemon=True)
        ros_thread.start()
        
        # Run GUI in main thread
        try:
            dashboard.run_gui()
        except KeyboardInterrupt:
            pass
    else:
        # Console mode
        try:
            rclpy.spin(dashboard)
        except KeyboardInterrupt:
            pass
    
    dashboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()