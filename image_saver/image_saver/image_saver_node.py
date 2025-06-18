#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # 声明参数
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('save_path', '~/turtlebot3_images')
        self.declare_parameter('save_interval', 1.0)  # 保存间隔（秒）
        self.declare_parameter('max_images', 100)     # 最大保存图片数量
        
        # 获取参数
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.save_path = os.path.expanduser(
            self.get_parameter('save_path').get_parameter_value().string_value
        )
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        
        # 创建保存目录
        os.makedirs(self.save_path, exist_ok=True)
        
        # 初始化变量
        self.bridge = CvBridge()
        self.image_count = 0
        self.last_save_time = self.get_clock().now()
        
        # 创建订阅者
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Image Saver Node initialized')
        self.get_logger().info(f'Subscribing to topic: {self.image_topic}')
        self.get_logger().info(f'Saving images to: {self.save_path}')
        self.get_logger().info(f'Save interval: {self.save_interval} seconds')
        self.get_logger().info(f'Max images: {self.max_images}')
        
    def image_callback(self, msg):
        try:
            # 检查是否应该保存图像
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_save_time).nanoseconds / 1e9
            
            if time_diff >= self.save_interval and self.image_count < self.max_images:
                # 将ROS图像消息转换为OpenCV格式，自动处理编码
                if msg.encoding == 'rgb8':
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # 生成文件名
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 去掉微秒的后3位
                filename = f"turtlebot3_image_{self.image_count:04d}_{timestamp}.jpg"
                filepath = os.path.join(self.save_path, filename)
                
                # 保存图像
                cv2.imwrite(filepath, cv_image)
                
                self.image_count += 1
                self.last_save_time = current_time
                
                self.get_logger().info(f'Saved image {self.image_count}/{self.max_images}: {filename}')
                
                # 如果达到最大图片数量，停止保存
                if self.image_count >= self.max_images:
                    self.get_logger().info(f'Reached maximum image count ({self.max_images}). Stopping image saving.')
                    
        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    image_saver_node = ImageSaverNode()
    
    try:
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
