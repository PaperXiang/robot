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

    // 速度平滑器 - 提升行走稳定性（借鉴论文AMP思想）
    class VelocitySmoother {
    private:
        std::deque<std::array<double, 3>> velocity_history_; // [vx, vy, vtheta]
        static constexpr int HISTORY_SIZE = 8;
        static constexpr double MAX_ACCEL = 0.8; // m/s² 最大加速度限制
        static constexpr double DT = 0.1; // 假设100ms控制周期
        
    public:
        std::array<double, 3> smooth(double vx, double vy, double vtheta) {
            velocity_history_.push_back({vx, vy, vtheta});
            if (velocity_history_.size() > HISTORY_SIZE) {
                velocity_history_.pop_front();
            }
            
            // 加权平均（越新的速度权重越大）
            std::array<double, 3> smoothed = {0, 0, 0};
            double total_weight = 0;
            for (size_t i = 0; i < velocity_history_.size(); i++) {
                double weight = (i + 1.0) / velocity_history_.size();
                smoothed[0] += weight * velocity_history_[i][0];
                smoothed[1] += weight * velocity_history_[i][1];
                smoothed[2] += weight * velocity_history_[i][2];
                total_weight += weight;
            }
            smoothed[0] /= total_weight;
            smoothed[1] /= total_weight;
            smoothed[2] /= total_weight;
            
            // 加速度限制（防止速度突变）
            if (velocity_history_.size() >= 2) {
                auto last = velocity_history_[velocity_history_.size() - 2];
                for (int i = 0; i < 3; i++) {
                    double delta = smoothed[i] - last[i];
                    if (std::fabs(delta) > MAX_ACCEL * DT) {
                        smoothed[i] = last[i] + std::copysign(MAX_ACCEL * DT, delta);
                    }
                }
            }
            
            return smoothed;
        }
        
        void reset() {
            velocity_history_.clear();
        }
    };
    
    VelocitySmoother velocity_smoother_;
};