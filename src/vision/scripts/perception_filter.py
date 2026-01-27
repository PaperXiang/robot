#!/usr/bin/env python3
"""
感知滤波模块 - 实现论文中的虚拟感知系统
基于《学习人形机器人的视觉驱动反应式足球技能》

功能:
1. 噪声注入 - 模拟真实感知的不确定性
2. 延迟模拟 - 模拟传感器延迟
3. 多帧融合 - 使用卡尔曼滤波提高鲁棒性
4. 检测率建模 - 根据距离/角度调整置信度
"""

import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple
import time


@dataclass
class Detection:
    """检测结果数据结构"""
    x: float  # 位置 x (米)
    y: float  # 位置 y (米)
    confidence: float  # 置信度 [0, 1]
    timestamp: float  # 时间戳
    class_id: int = 0  # 类别ID (0=球, 1=机器人, 2=球门等)
    
    
@dataclass
class FilterConfig:
    """滤波器配置参数"""
    # 噪声参数
    position_noise_std: float = 0.05  # 位置噪声标准差(米)
    confidence_noise_std: float = 0.02  # 置信度噪声标准差
    
    # 延迟参数
    latency_ms: float = 50.0  # 平均延迟(毫秒)
    latency_std_ms: float = 10.0  # 延迟标准差
    
    # 检测率参数
    detection_rate_near: float = 0.95  # 近距离检测率
    detection_rate_far: float = 0.70  # 远距离检测率
    distance_threshold: float = 5.0  # 近/远距离阈值(米)
    
    # 多帧融合参数
    fusion_window: int = 5  # 融合窗口大小
    kalman_process_noise: float = 0.01  # 过程噪声
    kalman_measurement_noise: float = 0.1  # 测量噪声
    
    # 预测参数
    max_prediction_time: float = 0.2  # 最大预测时间(秒)


class KalmanFilter2D:
    """2D卡尔曼滤波器用于位置跟踪"""
    
    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1):
        """
        初始化卡尔曼滤波器
        
        状态向量: [x, y, vx, vy]
        """
        self.dt = 0.033  # 假设30Hz更新率
        
        # 状态转移矩阵
        self.F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # 观测矩阵 (只观测位置)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # 过程噪声协方差
        self.Q = np.eye(4) * process_noise
        
        # 测量噪声协方差
        self.R = np.eye(2) * measurement_noise
        
        # 状态估计
        self.x = np.zeros(4)  # [x, y, vx, vy]
        
        # 协方差矩阵
        self.P = np.eye(4)
        
        # 是否已初始化
        self.initialized = False
        
    def predict(self, dt: Optional[float] = None):
        """预测步骤"""
        if dt is not None:
            # 更新状态转移矩阵
            F = self.F.copy()
            F[0, 2] = dt
            F[1, 3] = dt
        else:
            F = self.F
            
        # 预测状态
        self.x = F @ self.x
        
        # 预测协方差
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x[:2]  # 返回位置估计
        
    def update(self, measurement: np.ndarray):
        """更新步骤"""
        if not self.initialized:
            # 首次初始化
            self.x[:2] = measurement
            self.initialized = True
            return self.x[:2]
            
        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 更新状态
        y = measurement - self.H @ self.x  # 残差
        self.x = self.x + K @ y
        
        # 更新协方差
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x[:2]  # 返回位置估计
        
    def get_velocity(self) -> np.ndarray:
        """获取速度估计"""
        return self.x[2:]
        
    def get_position(self) -> np.ndarray:
        """获取位置估计"""
        return self.x[:2]


class PerceptionFilter:
    """
    感知滤波器 - 实现虚拟感知系统
    
    主要功能:
    1. 噪声建模: 模拟真实传感器的不确定性
    2. 延迟模拟: 模拟处理延迟
    3. 多帧融合: 卡尔曼滤波平滑轨迹
    4. 检测率建模: 根据距离调整检测概率
    """
    
    def __init__(self, config: Optional[FilterConfig] = None):
        """初始化感知滤波器"""
        self.config = config or FilterConfig()
        
        # 历史检测缓存
        self.history_buffer = deque(maxlen=self.config.fusion_window)
        
        # 卡尔曼滤波器 (每个目标一个)
        self.kalman_filters = {}  # {class_id: KalmanFilter2D}
        
        # 延迟队列
        self.delay_queue = deque()
        
        # 统计信息
        self.stats = {
            'total_detections': 0,
            'filtered_detections': 0,
            'avg_latency': 0.0,
            'dropped_by_distance': 0
        }
        
    def add_noise(self, detection: Detection, robot_velocity: float = 0.0) -> Detection:
        """
        添加噪声到检测结果
        
        模拟真实传感器的不确定性
        基于论文: 噪声随机器人运动速度增加 (Velocity-Dependent Noise)
        """
        # 基础噪声 + 速度依赖噪声
        # 假设速度每增加 1m/s, 噪声增加 50%
        velocity_factor = 1.0 + (abs(robot_velocity) * 0.5)
        
        current_pos_std = self.config.position_noise_std * velocity_factor
        current_conf_std = self.config.confidence_noise_std * velocity_factor
        
        # 位置噪声 (高斯噪声)
        noise_x = np.random.normal(0, current_pos_std)
        noise_y = np.random.normal(0, current_pos_std)
        
        # 置信度噪声
        noise_conf = np.random.normal(0, current_conf_std)
        
        return Detection(
            x=detection.x + noise_x,
            y=detection.y + noise_y,
            confidence=np.clip(detection.confidence + noise_conf, 0.0, 1.0),
            timestamp=detection.timestamp,
            class_id=detection.class_id
        )

        
    def simulate_latency(self, detection: Detection) -> Optional[Detection]:
        """
        模拟延迟
        
        将检测加入延迟队列,只返回延迟后的检测
        """
        # 生成延迟时间 (高斯分布)
        latency = np.random.normal(
            self.config.latency_ms / 1000.0,
            self.config.latency_std_ms / 1000.0
        )
        latency = max(0, latency)  # 确保非负
        
        # 加入延迟队列
        release_time = time.time() + latency
        self.delay_queue.append((release_time, detection))
        
        # 检查是否有到期的检测
        current_time = time.time()
        while self.delay_queue and self.delay_queue[0][0] <= current_time:
            _, delayed_detection = self.delay_queue.popleft()
            return delayed_detection
            
        return None
        
    def model_detection_rate(self, detection: Detection) -> bool:
        """
        检测率建模
        
        根据距离决定是否保留检测
        距离越远,检测率越低
        """
        distance = np.sqrt(detection.x**2 + detection.y**2)
        
        # 线性插值检测率
        if distance < self.config.distance_threshold:
            detection_rate = self.config.detection_rate_near
        else:
            # 远距离检测率随距离衰减
            ratio = (distance - self.config.distance_threshold) / self.config.distance_threshold
            detection_rate = self.config.detection_rate_near - \
                           (self.config.detection_rate_near - self.config.detection_rate_far) * \
                           min(ratio, 1.0)
        
        # 根据检测率随机决定是否保留
        if np.random.random() > detection_rate:
            self.stats['dropped_by_distance'] += 1
            return False
            
        return True
        
    def get_or_create_kalman(self, class_id: int) -> KalmanFilter2D:
        """获取或创建卡尔曼滤波器"""
        if class_id not in self.kalman_filters:
            self.kalman_filters[class_id] = KalmanFilter2D(
                process_noise=self.config.kalman_process_noise,
                measurement_noise=self.config.kalman_measurement_noise
            )
        return self.kalman_filters[class_id]
        
    def fuse_detections(self, detection: Detection) -> Detection:
        """
        多帧融合
        
        使用卡尔曼滤波平滑检测结果
        """
        # 获取对应的卡尔曼滤波器
        kf = self.get_or_create_kalman(detection.class_id)
        
        # 测量值
        measurement = np.array([detection.x, detection.y])
        
        # 更新卡尔曼滤波器
        filtered_pos = kf.update(measurement)
        
        # 返回滤波后的检测
        return Detection(
            x=filtered_pos[0],
            y=filtered_pos[1],
            confidence=detection.confidence,
            timestamp=detection.timestamp,
            class_id=detection.class_id
        )
        
    def predict_position(self, class_id: int, dt: float) -> Optional[np.ndarray]:
        """
        预测未来位置
        
        当检测丢失时,使用卡尔曼滤波器预测位置
        """
        if class_id not in self.kalman_filters:
            return None
            
        kf = self.kalman_filters[class_id]
        
        if dt > self.config.max_prediction_time:
            return None  # 预测时间过长,不可靠
            
        # 预测
        predicted_pos = kf.predict(dt)
        
        return predicted_pos
        
    def filter_detection(self, detection: Detection, 
                        robot_velocity: float = 0.0,
                        enable_noise: bool = True,
                        enable_latency: bool = False,
                        enable_detection_rate: bool = True,
                        enable_fusion: bool = True) -> Optional[Detection]:
        """
        完整的感知滤波流程
        
        Args:
            detection: 原始检测
            robot_velocity: 机器人当前速度 (m/s)
            enable_noise: 是否启用噪声模拟
            enable_latency: 是否启用延迟模拟
            enable_detection_rate: 是否启用检测率建模
            enable_fusion: 是否启用多帧融合
            
        Returns:
            滤波后的检测,如果被过滤掉则返回None
        """
        self.stats['total_detections'] += 1
        
        # 1. 检测率建模
        if enable_detection_rate and not self.model_detection_rate(detection):
            return None
            
        # 2. 添加噪声
        if enable_noise:
            detection = self.add_noise(detection, robot_velocity)
            
        # 3. 延迟模拟
        if enable_latency:
            detection = self.simulate_latency(detection)
            if detection is None:
                return None
                
        # 4. 多帧融合
        if enable_fusion:
            detection = self.fuse_detections(detection)
            
        # 5. 添加到历史缓存
        self.history_buffer.append(detection)
        
        self.stats['filtered_detections'] += 1
        
        return detection

        
    def get_velocity_estimate(self, class_id: int) -> Optional[np.ndarray]:
        """获取速度估计"""
        if class_id in self.kalman_filters:
            return self.kalman_filters[class_id].get_velocity()
        return None
        
    def reset(self, class_id: Optional[int] = None):
        """重置滤波器"""
        if class_id is None:
            # 重置所有
            self.kalman_filters.clear()
            self.history_buffer.clear()
            self.delay_queue.clear()
        else:
            # 重置特定目标
            if class_id in self.kalman_filters:
                del self.kalman_filters[class_id]
                
    def get_stats(self) -> dict:
        """获取统计信息"""
        return self.stats.copy()


# 使用示例
if __name__ == "__main__":
    # 创建滤波器
    config = FilterConfig(
        position_noise_std=0.05,
        latency_ms=50.0,
        detection_rate_near=0.95,
        detection_rate_far=0.70,
        fusion_window=5
    )
    
    filter = PerceptionFilter(config)
    
    # 模拟一系列检测
    print("模拟球检测序列:")
    for i in range(20):
        # 模拟球的运动 (直线运动)
        true_x = 2.0 + i * 0.1
        true_y = 1.0 + i * 0.05
        
        detection = Detection(
            x=true_x,
            y=true_y,
            confidence=0.9,
            timestamp=time.time(),
            class_id=0  # 球
        )
        
        # 滤波
        filtered = filter.filter_detection(
            detection,
            enable_noise=True,
            enable_latency=False,  # 示例中不启用延迟
            enable_detection_rate=True,
            enable_fusion=True
        )
        
        if filtered:
            print(f"Frame {i}: 原始=({true_x:.2f}, {true_y:.2f}), "
                  f"滤波后=({filtered.x:.2f}, {filtered.y:.2f})")
        else:
            print(f"Frame {i}: 检测被过滤")
            
    # 打印统计
    print("\n统计信息:")
    stats = filter.get_stats()
    for key, value in stats.items():
        print(f"  {key}: {value}")
        
    # 测试预测
    print("\n预测未来位置 (0.1秒后):")
    predicted = filter.predict_position(class_id=0, dt=0.1)
    if predicted is not None:
        print(f"  预测位置: ({predicted[0]:.2f}, {predicted[1]:.2f})")
