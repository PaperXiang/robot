#pragma once

/**
 * 运动原语库 - Motion Primitives
 * 基于论文的对抗性运动先验(AMP)思想
 * 
 * 功能:
 * 1. 多方向踢球动作选择
 * 2. 平滑轨迹生成
 * 3. 步态参数优化
 */

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace motion_primitives {

/**
 * 踢球方向枚举
 */
enum class KickDirection {
    FORWARD = 0,        // 正前方
    LEFT_45 = 45,       // 左前45度
    RIGHT_45 = -45,     // 右前45度
    LEFT_90 = 90,       // 左侧90度
    RIGHT_90 = -90,     // 右侧90度
    LEFT_30 = 30,       // 左前30度
    RIGHT_30 = -30      // 右前30度
};

/**
 * 踢球动作参数
 */
struct KickMotion {
    KickDirection direction;
    double power;           // 踢球力度 [0, 1]
    double preparation_time; // 准备时间(秒)
    std::string motion_name; // 动作名称
    
    KickMotion(KickDirection dir, double pwr = 0.8, double prep_time = 0.5)
        : direction(dir), power(pwr), preparation_time(prep_time) {
        motion_name = getMotionName(dir);
    }
    
private:
    std::string getMotionName(KickDirection dir) {
        switch(dir) {
            case KickDirection::FORWARD: return "kick_forward";
            case KickDirection::LEFT_45: return "kick_left_45";
            case KickDirection::RIGHT_45: return "kick_right_45";
            case KickDirection::LEFT_90: return "kick_left_90";
            case KickDirection::RIGHT_90: return "kick_right_90";
            case KickDirection::LEFT_30: return "kick_left_30";
            case KickDirection::RIGHT_30: return "kick_right_30";
            default: return "kick_forward";
        }
    }
};

/**
 * 姿态结构
 */
struct Pose {
    double x;
    double y;
    double theta;
    
    Pose(double x_ = 0, double y_ = 0, double theta_ = 0)
        : x(x_), y(y_), theta(theta_) {}
};

/**
 * 轨迹点
 */
struct TrajectoryPoint {
    Pose pose;
    double timestamp;
    double velocity;
    
    TrajectoryPoint(Pose p, double t, double v)
        : pose(p), timestamp(t), velocity(v) {}
};

/**
 * 轨迹
 */
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double total_time;
    
    void addPoint(const TrajectoryPoint& point) {
        points.push_back(point);
        if (!points.empty()) {
            total_time = points.back().timestamp;
        }
    }
    
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};

/**
 * 地形类型
 */
enum class TerrainType {
    GRASS,      // 草地
    CONCRETE,   // 混凝土
    SOIL,       // 土壤
    ASPHALT     // 沥青
};

/**
 * 步态参数
 */
struct GaitParams {
    double step_height;      // 抬腿高度
    double step_length;      // 步长
    double step_frequency;   // 步频
    double body_height;      // 身体高度
    double stability_margin; // 稳定性裕度
    
    GaitParams()
        : step_height(0.05), step_length(0.15), 
          step_frequency(1.5), body_height(0.7),
          stability_margin(0.02) {}
};

/**
 * 运动原语类
 */
class MotionPrimitives {
public:
    MotionPrimitives() 
        : kick_selection_threshold_(15.0),  // 15度阈值
          trajectory_smoothing_enabled_(true),
          smoothing_window_(3) {}
    
    /**
     * 根据目标角度选择最优踢球动作
     * 
     * @param target_angle 目标角度(度数, 相对于机器人前方)
     * @param distance_to_goal 到球门的距离(米) - 用于能量自适应
     * @return 最优踢球动作
     */
    KickMotion selectKick(double target_angle, double distance_to_goal = 3.0) {
        // 归一化角度到[-180, 180]
        target_angle = normalizeAngle(target_angle);
        
        // 可用的踢球方向
        std::vector<KickDirection> available_kicks = {
            KickDirection::FORWARD,
            KickDirection::LEFT_30,
            KickDirection::RIGHT_30,
            KickDirection::LEFT_45,
            KickDirection::RIGHT_45,
            KickDirection::LEFT_90,
            KickDirection::RIGHT_90
        };
        
        // 找到最接近的方向
        KickDirection best_direction = KickDirection::FORWARD;
        double min_diff = 180.0;
        
        for (const auto& kick_dir : available_kicks) {
            double dir_angle = static_cast<double>(kick_dir);
            double diff = std::abs(normalizeAngle(target_angle - dir_angle));
            
            if (diff < min_diff) {
                min_diff = diff;
                best_direction = kick_dir;
            }
        }
        
        // 如果差异太大,选择正前方并调整身体朝向
        if (min_diff > kick_selection_threshold_) {
            best_direction = KickDirection::FORWARD;
        }
        
        // 能量自适应 (Energy-Adaptive Kicking)
        // 距离越近，力度越小，以节省能量并提高稳定性
        // 假设最大力度为 1.0, 最小力度为 0.3
        double adaptive_power = std::min(1.0, std::max(0.3, distance_to_goal * 0.2));
        
        // 对短距离(如传球)，进一步降低准备时间
        double prep_time = (adaptive_power < 0.5) ? 0.3 : 0.5;
        
        return KickMotion(best_direction, adaptive_power, prep_time);
    }

    
    /**
     * 生成平滑轨迹
     * 
     * @param start 起始姿态
     * @param goal 目标姿态
     * @param duration 轨迹持续时间(秒)
     * @param num_points 轨迹点数量
     * @return 平滑轨迹
     */
    Trajectory generateSmoothPath(const Pose& start, const Pose& goal, 
                                 double duration = 2.0, int num_points = 20) {
        Trajectory traj;
        
        if (!trajectory_smoothing_enabled_) {
            // 直接线性插值
            for (int i = 0; i <= num_points; ++i) {
                double t = static_cast<double>(i) / num_points;
                Pose p = interpolatePose(start, goal, t);
                traj.addPoint(TrajectoryPoint(p, t * duration, 0.5));
            }
            return traj;
        }
        
        // 使用三次样条插值生成平滑轨迹
        std::vector<double> times;
        std::vector<Pose> poses;
        
        // 控制点
        times.push_back(0.0);
        poses.push_back(start);
        
        // 中间点(可以添加更多控制点以避障)
        Pose mid = interpolatePose(start, goal, 0.5);
        times.push_back(duration * 0.5);
        poses.push_back(mid);
        
        times.push_back(duration);
        poses.push_back(goal);
        
        // 生成平滑轨迹点
        for (int i = 0; i <= num_points; ++i) {
            double t = static_cast<double>(i) / num_points * duration;
            Pose p = cubicSplineInterpolation(times, poses, t);
            
            // 计算速度(简化版)
            double velocity = 0.5;
            if (i > 0 && i < num_points) {
                velocity = 0.7;  // 中间段加速
            }
            
            traj.addPoint(TrajectoryPoint(p, t, velocity));
        }
        
        return traj;
    }
    
    /**
     * 根据地形优化步态参数
     * 
     * @param terrain 地形类型
     * @return 优化后的步态参数
     */
    GaitParams optimizeGait(TerrainType terrain) {
        GaitParams params;
        
        switch(terrain) {
            case TerrainType::GRASS:
                // 草地: 较高抬腿,较短步长
                params.step_height = 0.06;
                params.step_length = 0.12;
                params.step_frequency = 1.4;
                params.stability_margin = 0.025;
                break;
                
            case TerrainType::CONCRETE:
                // 混凝土: 标准参数
                params.step_height = 0.05;
                params.step_length = 0.15;
                params.step_frequency = 1.5;
                params.stability_margin = 0.02;
                break;
                
            case TerrainType::SOIL:
                // 土壤: 更高抬腿,更短步长,更慢步频
                params.step_height = 0.07;
                params.step_length = 0.10;
                params.step_frequency = 1.2;
                params.stability_margin = 0.03;
                break;
                
            case TerrainType::ASPHALT:
                // 沥青: 较低抬腿,较长步长
                params.step_height = 0.04;
                params.step_length = 0.18;
                params.step_frequency = 1.6;
                params.stability_margin = 0.015;
                break;
        }
        
        return params;
    }
    
    /**
     * 应用运动平滑滤波
     * 
     * @param trajectory 原始轨迹
     * @return 平滑后的轨迹
     */
    Trajectory smoothTrajectory(const Trajectory& trajectory) {
        if (!trajectory_smoothing_enabled_ || trajectory.size() < smoothing_window_) {
            return trajectory;
        }
        
        Trajectory smoothed;
        
        for (size_t i = 0; i < trajectory.points.size(); ++i) {
            // 计算窗口范围
            int half_window = smoothing_window_ / 2;
            int start_idx = std::max(0, static_cast<int>(i) - half_window);
            int end_idx = std::min(static_cast<int>(trajectory.points.size()) - 1, 
                                  static_cast<int>(i) + half_window);
            
            // 平均滤波
            double avg_x = 0, avg_y = 0, avg_theta = 0;
            int count = 0;
            
            for (int j = start_idx; j <= end_idx; ++j) {
                avg_x += trajectory.points[j].pose.x;
                avg_y += trajectory.points[j].pose.y;
                avg_theta += trajectory.points[j].pose.theta;
                count++;
            }
            
            Pose smoothed_pose(avg_x / count, avg_y / count, avg_theta / count);
            smoothed.addPoint(TrajectoryPoint(
                smoothed_pose,
                trajectory.points[i].timestamp,
                trajectory.points[i].velocity
            ));
        }
        
        return smoothed;
    }
    
    // 设置参数
    void setKickSelectionThreshold(double threshold) {
        kick_selection_threshold_ = threshold;
    }
    
    void enableTrajectorySmoothing(bool enable) {
        trajectory_smoothing_enabled_ = enable;
    }
    
    void setSmoothingWindow(int window) {
        smoothing_window_ = window;
    }
    
private:
    double kick_selection_threshold_;  // 踢球方向选择阈值(度)
    bool trajectory_smoothing_enabled_; // 是否启用轨迹平滑
    int smoothing_window_;              // 平滑窗口大小
    
    /**
     * 归一化角度到[-180, 180]
     */
    double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
    
    /**
     * 线性插值姿态
     */
    Pose interpolatePose(const Pose& start, const Pose& goal, double t) {
        return Pose(
            start.x + t * (goal.x - start.x),
            start.y + t * (goal.y - start.y),
            start.theta + t * normalizeAngle(goal.theta - start.theta)
        );
    }
    
    /**
     * 三次样条插值
     * 简化版本,实际应用中可以使用更复杂的样条算法
     */
    Pose cubicSplineInterpolation(const std::vector<double>& times,
                                  const std::vector<Pose>& poses,
                                  double t) {
        // 找到t所在的区间
        size_t idx = 0;
        for (size_t i = 0; i < times.size() - 1; ++i) {
            if (t >= times[i] && t <= times[i + 1]) {
                idx = i;
                break;
            }
        }
        
        // 局部归一化时间
        double t_local = (t - times[idx]) / (times[idx + 1] - times[idx]);
        
        // Hermite插值(平滑)
        double h00 = 2 * t_local * t_local * t_local - 3 * t_local * t_local + 1;
        double h10 = t_local * t_local * t_local - 2 * t_local * t_local + t_local;
        double h01 = -2 * t_local * t_local * t_local + 3 * t_local * t_local;
        double h11 = t_local * t_local * t_local - t_local * t_local;
        
        // 切线(简化为零,可以改进)
        double m0 = 0, m1 = 0;
        
        Pose result;
        result.x = h00 * poses[idx].x + h01 * poses[idx + 1].x;
        result.y = h00 * poses[idx].y + h01 * poses[idx + 1].y;
        result.theta = h00 * poses[idx].theta + h01 * poses[idx + 1].theta;
        
        return result;
    }
};

} // namespace motion_primitives
