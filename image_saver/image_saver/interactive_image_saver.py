#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class InteractiveImageSaver(Node):
    def __init__(self):
        super().__init__('interactive_image_saver')
        
        # 声明参数
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('save_path', '~/turtlebot3_images')
        self.declare_parameter('show_image', True)
        
        # 获取参数
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.save_path = os.path.expanduser(
            self.get_parameter('save_path').get_parameter_value().string_value
        )
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        
        # 创建保存目录
        os.makedirs(self.save_path, exist_ok=True)
        
        # 初始化变量
        self.bridge = CvBridge()
        self.image_count = 0
        self.current_image = None
        
        # 创建订阅者
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # 创建保存命令订阅者
        self.save_command_subscription = self.create_subscription(
            String,
            '/save_image_command',
            self.save_command_callback,
            10
        )
        
        # 创建定时器用于更新显示
        if self.show_image:
            self.timer = self.create_timer(0.1, self.update_display)
        
        self.get_logger().info(f'Interactive Image Saver initialized')
        self.get_logger().info(f'Subscribing to topic: {self.image_topic}')
        self.get_logger().info(f'Saving images to: {self.save_path}')
        self.get_logger().info(f'Show image: {self.show_image}')
        
        if self.show_image:
            self.get_logger().info('Press "s" in the image window to save, "q" to quit')
        
        self.get_logger().info('You can also publish to /save_image_command topic to save images programmatically')
        
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式，自动处理编码
            if msg.encoding == 'rgb8':
                self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
    
    def save_command_callback(self, msg):
        """处理保存命令"""
        if msg.data.lower() == 'save':
            self.save_current_image()
    
    def save_current_image(self):
        """保存当前图像"""
        if self.current_image is not None:
            try:
                # 生成文件名
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = f"turtlebot3_manual_{self.image_count:04d}_{timestamp}.jpg"
                filepath = os.path.join(self.save_path, filename)
                
                # 保存图像
                cv2.imwrite(filepath, self.current_image)
                
                self.image_count += 1
                self.get_logger().info(f'Saved image: {filename}')
                
                return True
            except Exception as e:
                self.get_logger().error(f'Error saving image: {str(e)}')
                return False
        else:
            self.get_logger().warn('No image available to save')
            return False
    
    def update_display(self):
        """更新图像显示"""
        if self.current_image is not None:
            # 在图像上添加文本提示
            display_image = self.current_image.copy()
            
            # 添加保存计数和提示信息
            cv2.putText(display_image, f'Images saved: {self.image_count}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_image, 'Press "s" to save, "q" to quit', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示图像
            cv2.imshow('TurtleBot3 Camera Feed', display_image)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.save_current_image()
            elif key == ord('q'):
                self.get_logger().info('Quit requested by user')
                cv2.destroyAllWindows()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    interactive_saver = InteractiveImageSaver()
    
    try:
        rclpy.spin(interactive_saver)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        interactive_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
