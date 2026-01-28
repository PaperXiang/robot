#pragma once

#include <iostream>
#include <string>
#include <deque>
#include <array>
#include <cmath>
#include <rerun.hpp>

#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "booster_msgs/msg/rpc_req_msg.hpp"

using namespace std;

class Brain; // 类相互依赖，向前声明


/**
 * RobotClient 类，调用 RobotSDK 操控机器人的操作都放在这里
 * 因为目前的代码里依赖 brain 里相关的一些东西，现在设计成跟 brain 相互依赖
 */
class RobotClient
{
public:
    RobotClient(Brain* argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief 
     *
     * @param pitch
     * @param yaw
     *
     * @return int , 0 表示执行成功
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief 
     * 
     * @param x double, 
     * @param y double, 
     * @param theta double, 
     * @param applyMinX, applyMinY, applyMinTheta bool 
     * 
     * @return int , 0 表示执行成功
     * 
    */
    int setVelocity(double x, double y, double theta, bool applyMinX=true, bool applyMinY=true, bool applyMinTheta=true);

    int crabWalk(double angle, double speed);

    /**
     * @brief 
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double,
     * @param avoidObstacle bool, 
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief   
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double,
     * @param avoidObstacle bool, 
     * 
     * @return int 运控命令返回值, 0 代表成功
     */

    int moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);
    /**
     * @brief 
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double, 
     * @param avoidObstacle bool, 
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief 挥手
     */
    int waveHand(bool doWaveHand);

    /**
     * @brief 起身
     */
    int standUp();

    /**
     * @brief 快速起身 - 优化起身速度
     * @details 通过预调整姿态加速起身过程，适用于比赛中需要快速恢复的场景
     * @return int , 0 表示执行成功
     */
    int quickStandUp();

    int enterDamping();

    double msecsToCollide(double vx, double vy, double vtheta, double maxTime=10000);

    bool isStandingStill(double timeBuffer = 1000);

private:
    int call(booster_interface::msg::BoosterApiReqMsg msg);
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
    double _vx, _vy, _vtheta;
    rclcpp::Time _lastCmdTime;
    rclcpp::Time _lastNonZeroCmdTime;

    // 速度平滑器 - 提升行走稳定性（借鉴论文AMP思想，优化参数版本）
    class VelocitySmoother {
    private:
        std::deque<std::array<double, 3>> velocity_history_; // [vx, vy, vtheta]
        std::array<double, 3> last_output_ = {0, 0, 0};      // 上一次输出，用于指数平滑
        
        // 优化后的参数（保持高速同时提升稳定性）
        static constexpr int HISTORY_SIZE = 10;              // 增加历史窗口以获得更平滑的输出
        static constexpr double MAX_ACCEL_LINEAR = 0.6;      // m/s² 线性速度最大加速度（降低以提升稳定性）
        static constexpr double MAX_ACCEL_ANGULAR = 0.8;     // rad/s² 角速度最大加速度（保持响应性）
        static constexpr double DT = 0.1;                    // 100ms控制周期
        static constexpr double SMOOTH_FACTOR = 0.85;        // 指数平滑因子（0-1，越大越平滑）
        
    public:
        std::array<double, 3> smooth(double vx, double vy, double vtheta) {
            velocity_history_.push_back({vx, vy, vtheta});
            if (velocity_history_.size() > HISTORY_SIZE) {
                velocity_history_.pop_front();
            }
            
            // 加权平均（越新的速度权重越大，使用二次权重提升近期值的影响）
            std::array<double, 3> smoothed = {0, 0, 0};
            double total_weight = 0;
            for (size_t i = 0; i < velocity_history_.size(); i++) {
                // 使用二次加权，使最新值的权重更大
                double weight = std::pow((i + 1.0) / velocity_history_.size(), 2);
                smoothed[0] += weight * velocity_history_[i][0];
                smoothed[1] += weight * velocity_history_[i][1];
                smoothed[2] += weight * velocity_history_[i][2];
                total_weight += weight;
            }
            smoothed[0] /= total_weight;
            smoothed[1] /= total_weight;
            smoothed[2] /= total_weight;
            
            // 指数移动平均（进一步平滑输出）
            smoothed[0] = SMOOTH_FACTOR * last_output_[0] + (1.0 - SMOOTH_FACTOR) * smoothed[0];
            smoothed[1] = SMOOTH_FACTOR * last_output_[1] + (1.0 - SMOOTH_FACTOR) * smoothed[1];
            smoothed[2] = SMOOTH_FACTOR * last_output_[2] + (1.0 - SMOOTH_FACTOR) * smoothed[2];
            
            // 加速度限制（防止速度突变，线性和角速度分开限制）
            if (velocity_history_.size() >= 2) {
                // 线性速度加速度限制
                for (int i = 0; i < 2; i++) {
                    double delta = smoothed[i] - last_output_[i];
                    if (std::fabs(delta) > MAX_ACCEL_LINEAR * DT) {
                        smoothed[i] = last_output_[i] + std::copysign(MAX_ACCEL_LINEAR * DT, delta);
                    }
                }
                // 角速度加速度限制
                double delta_theta = smoothed[2] - last_output_[2];
                if (std::fabs(delta_theta) > MAX_ACCEL_ANGULAR * DT) {
                    smoothed[2] = last_output_[2] + std::copysign(MAX_ACCEL_ANGULAR * DT, delta_theta);
                }
            }
            
            last_output_ = smoothed;
            return smoothed;
        }
        
        void reset() {
            velocity_history_.clear();
            last_output_ = {0, 0, 0};
        }
    };
    
    VelocitySmoother velocity_smoother_;
};