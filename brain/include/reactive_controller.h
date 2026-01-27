#pragma once

/**
 * 反应式控制器 - Reactive Controller
 * 基于论文的感知-动作耦合思想
 * 
 * 功能:
 * 1. 直接从感知到动作的快速映射
 * 2. 主动视觉搜索
 * 3. 历史观测编码
 * 4. 球追踪和预测
 */

#include <deque>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "brain_data.h"
#include "motion_primitives.h"

/**
 * 观测数据结构
 */
struct Observation {
    double ball_x;          // 球的x位置(机器人坐标系)
    double ball_y;          // 球的y位置
    double ball_confidence; // 球的置信度
    bool ball_visible;      // 球是否可见
    
    double robot_x;         // 机器人自身位置
    double robot_y;
    double robot_theta;
    
    double head_pitch;      // 头部姿态
    double head_yaw;
    
    double timestamp;       // 时间戳
    
    Observation()
        : ball_x(0), ball_y(0), ball_confidence(0), ball_visible(false),
          robot_x(0), robot_y(0), robot_theta(0),
          head_pitch(0), head_yaw(0), timestamp(0) {}
};

/**
 * 运动命令
 */
struct MotionCommand {
    double linear_x;    // 前进速度
    double linear_y;    // 侧向速度
    double angular_z;   // 旋转速度
    bool should_kick;   // 是否踢球
    motion_primitives::KickDirection kick_direction;
    
    MotionCommand()
        : linear_x(0), linear_y(0), angular_z(0), 
          should_kick(false), 
          kick_direction(motion_primitives::KickDirection::FORWARD) {}
};

/**
 * 头部控制命令
 */
struct HeadCommand {
    double pitch;       // 俯仰角
    double yaw;         // 偏航角
    double speed;       // 运动速度
    
    HeadCommand(double p = 0, double y = 0, double s = 1.0)
        : pitch(p), yaw(y), speed(s) {}
};

/**
 * 观测编码器
 * 使用历史观测恢复部分可观测信息
 * 基于论文: 扩展历史长度到50帧以处理噪声
 */
class ObservationEncoder {
public:
    ObservationEncoder(int history_length = 50) // 增加到50帧 (约1秒@50Hz)
        : history_length_(history_length) {}
    
    /**
     * 添加新的观测
     */
    void addObservation(const Observation& obs) {
        history_.push_back(obs);
        if (history_.size() > history_length_) {
            history_.pop_front();
        }
    }
    
    /**
     * 编码历史观测为特征向量
     * 
     * 特征包括:
     * - 球的位置历史
     * - 球的速度估计
     * - 球的可见性历史
     * - 机器人的运动历史
     */
    Eigen::VectorXd encode() {
        if (history_.empty()) {
            return Eigen::VectorXd::Zero(20);  // 默认特征维度
        }
        
        // 简化版本: 提取关键特征
        Eigen::VectorXd features(20);
        features.setZero();
        
        // 当前球位置
        if (!history_.empty()) {
            const auto& latest = history_.back();
            features(0) = latest.ball_x;
            features(1) = latest.ball_y;
            features(2) = latest.ball_confidence;
            features(3) = latest.ball_visible ? 1.0 : 0.0;
        }
        
        // 球速度估计 (使用过去10帧平均以抵抗噪声)
        if (history_.size() >= 10) {
            const auto& curr = history_.back();
            const auto& prev = history_[history_.size() - 10];
            double dt = curr.timestamp - prev.timestamp;
            
            if (dt > 0 && curr.ball_visible && prev.ball_visible) {
                features(4) = (curr.ball_x - prev.ball_x) / dt;  // vx
                features(5) = (curr.ball_y - prev.ball_y) / dt;  // vy
            }
        }
        
        // 球可见性统计 (长期)
        int visible_count = 0;
        for (const auto& obs : history_) {
            if (obs.ball_visible) visible_count++;
        }
        features(6) = static_cast<double>(visible_count) / history_.size();
        
        // 机器人状态
        if (!history_.empty()) {
            const auto& latest = history_.back();
            features(7) = latest.robot_x;
            features(8) = latest.robot_y;
            features(9) = latest.robot_theta;
            features(10) = latest.head_pitch;
            features(11) = latest.head_yaw;
        }
        
        return features;
    }

    
    /**
     * 预测球的未来位置
     * 
     * @param dt 预测时间(秒)
     * @return 预测的球位置 (x, y)
     */
    std::pair<double, double> predictBallPosition(double dt) {
        if (history_.size() < 2) {
            // 没有足够历史,返回当前位置
            if (!history_.empty()) {
                return {history_.back().ball_x, history_.back().ball_y};
            }
            return {0, 0};
        }
        
        // 简单的线性预测
        const auto& curr = history_.back();
        const auto& prev = history_[history_.size() - 2];
        double dt_hist = curr.timestamp - prev.timestamp;
        
        if (dt_hist > 0 && curr.ball_visible && prev.ball_visible) {
            double vx = (curr.ball_x - prev.ball_x) / dt_hist;
            double vy = (curr.ball_y - prev.ball_y) / dt_hist;
            
            return {
                curr.ball_x + vx * dt,
                curr.ball_y + vy * dt
            };
        }
        
        return {curr.ball_x, curr.ball_y};
    }
    
    /**
     * 获取球丢失时间
     */
    double getTimeSinceBallLost() {
        if (history_.empty()) return 999.0;
        
        for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
            if (it->ball_visible) {
                return history_.back().timestamp - it->timestamp;
            }
        }
        
        return 999.0;  // 很久没见到球了
    }
    
    void reset() {
        history_.clear();
    }
    
private:
    std::deque<Observation> history_;
    size_t history_length_;
};

/**
 * 反应式控制器
 */
class ReactiveController {
public:
    ReactiveController()
        : encoder_(50),
          ball_track_distance_(3.0),
          reaction_time_threshold_(0.1),
          search_pattern_index_(0) {}
    
    /**
     * 对球做出反应 - 核心反应式控制
     * 
     * @param ball_detection 球的检测信息
     * @param robot_state 机器人状态
     * @return 运动命令
     */
    MotionCommand reactToBall(const GameObject& ball, const RobotState& robot_state) {
        MotionCommand cmd;
        
        // 计算球的距离
        double ball_dist = std::sqrt(ball.relativePos.x * ball.relativePos.x + 
                                    ball.relativePos.y * ball.relativePos.y);
        
        // 计算球的角度
        double ball_angle = std::atan2(ball.relativePos.y, ball.relativePos.x);
        
        // 反应式策略: 根据距离和角度直接生成命令
        if (ball_dist < 0.3) {
            // 非常近 - 准备踢球
            cmd.should_kick = true;
            
            // 使用运动原语选择踢球方向
            motion_primitives::MotionPrimitives mp;
            double target_angle = calculateKickTargetAngle(ball, robot_state);
            auto kick_motion = mp.selectKick(target_angle);
            cmd.kick_direction = kick_motion.direction;
            
            // 微调位置
            cmd.linear_x = 0.1;
            cmd.angular_z = ball_angle * 0.5;
            
        } else if (ball_dist < ball_track_distance_) {
            // 中等距离 - 快速接近
            cmd.linear_x = std::min(0.5, ball_dist * 0.3);  // 速度与距离成正比
            cmd.linear_y = ball.relativePos.y * 0.2;        // 侧向调整
            cmd.angular_z = ball_angle * 1.0;               // 对准球
            
        } else {
            // 较远 - 正常接近
            cmd.linear_x = 0.3;
            cmd.angular_z = ball_angle * 0.8;
        }
        
        // 应用平滑约束 (Reference: N-P3O Constraint)
        cmd = applyActionSmoothing(cmd);
        
        return cmd;
    }

private:
    ObservationEncoder encoder_;
    double ball_track_distance_;      // 反应式追踪距离阈值
    double reaction_time_threshold_;  // 反应时间阈值
    int search_pattern_index_;        // 搜索模式索引
    double last_search_switch_time_;  // 上次切换搜索位置的时间
    MotionCommand last_cmd_;          // 上一次的命令,用于平滑
    
    /**
     * 应用动作平滑 (低通滤波/速率限制)
     * 模拟 N-P3O 的平滑效果,减少机械磨损
     */
    MotionCommand applyActionSmoothing(MotionCommand target_cmd) {
        MotionCommand smoothed = target_cmd;
        
        // 平滑参数 (0.0 = 完全不变, 1.0 = 完全使用新值)
        // 较低的值意味着更平滑但响应更慢
        const double alpha_linear = 0.2; 
        const double alpha_angular = 0.3;
        
        smoothed.linear_x = last_cmd_.linear_x * (1.0 - alpha_linear) + target_cmd.linear_x * alpha_linear;
        smoothed.linear_y = last_cmd_.linear_y * (1.0 - alpha_linear) + target_cmd.linear_y * alpha_linear;
        smoothed.angular_z = last_cmd_.angular_z * (1.0 - alpha_angular) + target_cmd.angular_z * alpha_angular;
        
        // 踢球指令不需要平滑，但需要防抖
        smoothed.should_kick = target_cmd.should_kick;
        smoothed.kick_direction = target_cmd.kick_direction;
        
        last_cmd_ = smoothed;
        return smoothed;
    }

     * 
     * @param robot_state 机器人状态
     * @return 头部控制命令
     */
    HeadCommand searchBall(const RobotState& robot_state) {
        HeadCommand head_cmd;
        
        double time_since_lost = encoder_.getTimeSinceBallLost();
        
        if (time_since_lost < 0.5) {
            // 刚丢失 - 预测位置
            auto predicted_pos = encoder_.predictBallPosition(time_since_lost);
            
            // 计算头部应该看向的角度
            double target_yaw = std::atan2(predicted_pos.second, predicted_pos.first);
            double target_pitch = -0.3;  // 向下看
            
            head_cmd.yaw = target_yaw;
            head_cmd.pitch = target_pitch;
            head_cmd.speed = 2.0;  // 快速转向
            
        } else {
            // 丢失较久 - 系统性搜索
            // 使用搜索模式: 左-中-右-下
            const std::vector<std::pair<double, double>> search_pattern = {
                {0.0, -0.3},    // 中间,向下
                {0.5, -0.3},    // 左边,向下
                {-0.5, -0.3},   // 右边,向下
                {0.0, -0.5},    // 中间,更向下
                {0.7, -0.2},    // 左边,稍向下
                {-0.7, -0.2}    // 右边,稍向下
            };
            
            // 循环搜索模式
            auto pattern = search_pattern[search_pattern_index_ % search_pattern.size()];
            head_cmd.yaw = pattern.first;
            head_cmd.pitch = pattern.second;
            head_cmd.speed = 1.5;
            
            // 每秒切换一次搜索位置
            if (time_since_lost > last_search_switch_time_ + 1.0) {
                search_pattern_index_++;
                last_search_switch_time_ = time_since_lost;
            }
        }
        
        return head_cmd;
    }
    
    /**
     * 更新观测历史
     */
    void updateObservation(const Observation& obs) {
        encoder_.addObservation(obs);
    }
    
    /**
     * 判断是否应该使用反应式控制
     * 
     * 当球在近距离且可见时,使用反应式控制以减少延迟
     */
    bool shouldUseReactiveControl(const GameObject& ball) {
        double ball_dist = std::sqrt(ball.relativePos.x * ball.relativePos.x + 
                                    ball.relativePos.y * ball.relativePos.y);
        
        return ball.visible && ball_dist < ball_track_distance_;
    }
    
    void reset() {
        encoder_.reset();
        search_pattern_index_ = 0;
        last_search_switch_time_ = 0;
    }
    
    // 参数设置
    void setBallTrackDistance(double dist) {
        ball_track_distance_ = dist;
    }
    
    void setReactionTimeThreshold(double threshold) {
        reaction_time_threshold_ = threshold;
    }
    
private:
    ObservationEncoder encoder_;
    double ball_track_distance_;      // 反应式追踪距离阈值
    double reaction_time_threshold_;  // 反应时间阈值
    int search_pattern_index_;        // 搜索模式索引
    double last_search_switch_time_;  // 上次切换搜索位置的时间
    
    /**
     * 计算踢球目标角度
     * 简化版本,实际应考虑球门位置、对手位置等
     */
    double calculateKickTargetAngle(const GameObject& ball, const RobotState& robot_state) {
        // 简化: 朝向对方球门
        // 实际应该从BrainData中获取球门位置
        return 0.0;  // 正前方
    }
};
