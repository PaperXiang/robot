#!/usr/bin/env python3
"""
ROS2感知滤波器包装器
将perception_filter.py集成到ROS2系统中

订阅原始检测消息,应用滤波后发布优化的检测结果
"""

import rclpy
from rclpy.node import Node
from vision_interface.msg import Detections, Detection as DetectionMsg
from booster_interface.msg import Odometer  # 导入里程计消息
from std_msgs.msg import Header
import numpy as np
from perception_filter import PerceptionFilter, FilterConfig, Detection
import time


class PerceptionFilterNode(Node):
    """ROS2感知滤波器节点"""
    
    def __init__(self):
        super().__init__('perception_filter_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('position_noise_std', 0.05),
                ('latency_ms', 50.0),
                ('detection_rate_near', 0.95),
                ('detection_rate_far', 0.70),
                ('distance_threshold', 5.0),
                ('fusion_window', 5),
                ('kalman_process_noise', 0.01),
                ('kalman_measurement_noise', 0.1),
                ('enable_noise', True),
                ('enable_latency', False),  # 默认关闭延迟模拟(真实系统已有延迟)
                ('enable_detection_rate', True),
                ('enable_fusion', True),
            ]
        )
        
        # 获取参数
        config = FilterConfig(
            position_noise_std=self.get_parameter('position_noise_std').value,
            latency_ms=self.get_parameter('latency_ms').value,
            detection_rate_near=self.get_parameter('detection_rate_near').value,
            detection_rate_far=self.get_parameter('detection_rate_far').value,
            distance_threshold=self.get_parameter('distance_threshold').value,
            fusion_window=self.get_parameter('fusion_window').value,
            kalman_process_noise=self.get_parameter('kalman_process_noise').value,
            kalman_measurement_noise=self.get_parameter('kalman_measurement_noise').value,
        )
        
        # 创建滤波器
        self.filter = PerceptionFilter(config)
        
        # 滤波开关
        self.enable_noise = self.get_parameter('enable_noise').value
        self.enable_latency = self.get_parameter('enable_latency').value
        self.enable_detection_rate = self.get_parameter('enable_detection_rate').value
        self.enable_fusion = self.get_parameter('enable_fusion').value
        
        # 机器人当前速度
        self.current_velocity = 0.0
        
        # 订阅里程计信息 (获取速度)
        self.odom_subscription = self.create_subscription(
            Odometer,
            '/booster_interface/odometer',  # 假设的话题名，需要确认
            self.odom_callback,
            10
        )
        
        # 订阅原始检测
        self.subscription = self.create_subscription(
            Detections,
            '/booster_vision/detection',
            self.detection_callback,
            10
        )
        
        # 发布滤波后的检测
        self.publisher = self.create_publisher(
            Detections,
            '/booster_vision/detection_filtered',
            10
        )
        
        # 统计定时器
        self.stats_timer = self.create_timer(10.0, self.print_statistics)
        
        self.get_logger().info('Perception Filter Node initialized')
        self.get_logger().info(f'  Noise: {self.enable_noise}')
        self.get_logger().info(f'  Latency: {self.enable_latency}')
        self.get_logger().info(f'  Detection Rate: {self.enable_detection_rate}')
        self.get_logger().info(f'  Fusion: {self.enable_fusion}')
        
    def odom_callback(self, msg: Odometer):
        """里程计回调，更新速度信息"""
        # 计算线速度大小
        self.current_velocity = np.sqrt(msg.vx**2 + msg.vy**2)
        
    def detection_callback(self, msg: Detections):
        """检测消息回调"""
        # 创建输出消息
        filtered_msg = Detections()
        filtered_msg.header = msg.header
        
        # 处理每个检测
        for det in msg.detections:
            # 转换为内部格式
            detection = Detection(
                x=det.bbox.center.position.x,
                y=det.bbox.center.position.y,
                confidence=det.results[0].hypothesis.score if det.results else 0.0,
                timestamp=time.time(),
                class_id=det.results[0].hypothesis.class_id.encode() if det.results else 0
            )
            
            # 应用滤波 (传入当前速度)
            filtered = self.filter.filter_detection(
                detection,
                robot_velocity=self.current_velocity,
                enable_noise=self.enable_noise,
                enable_latency=self.enable_latency,
                enable_detection_rate=self.enable_detection_rate,
                enable_fusion=self.enable_fusion
            )
            
            # 如果通过滤波,添加到输出
            if filtered is not None:
                # 转换回ROS消息格式
                filtered_det = DetectionMsg()
                filtered_det.bbox.center.position.x = filtered.x
                filtered_det.bbox.center.position.y = filtered.y
                
                # 保留原始消息的其他字段
                filtered_det.bbox.center.position.z = det.bbox.center.position.z
                filtered_det.bbox.center.theta = det.bbox.center.theta
                filtered_det.bbox.size = det.bbox.size
                
                # 更新置信度
                if det.results:
                    filtered_det.results = det.results
                    filtered_det.results[0].hypothesis.score = filtered.confidence
                
                filtered_msg.detections.append(filtered_det)
        
        # 发布滤波后的消息
        self.publisher.publish(filtered_msg)
        
    def print_statistics(self):
        """打印统计信息"""
        stats = self.filter.get_stats()
        
        if stats['total_detections'] > 0:
            filter_rate = stats['filtered_detections'] / stats['total_detections'] * 100
            drop_rate = stats['dropped_by_distance'] / stats['total_detections'] * 100
            
            self.get_logger().info(
                f"Stats - Total: {stats['total_detections']}, "
                f"Filtered: {stats['filtered_detections']} ({filter_rate:.1f}%), "
                f"Dropped: {stats['dropped_by_distance']} ({drop_rate:.1f}%), "
                f"CurVel: {self.current_velocity:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
