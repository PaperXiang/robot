# 足球机器人跌倒恢复策略 - 配置和故障排查指南

## 1. 配置示例

### 1.1 config.yaml 中的完整配置

```yaml
# ==================== 恢复策略配置 ====================

# 恢复相关基础配置
recovery:
  # 最大重试次数 (次)
  # - 设置过低：容易失败后无法恢复
  # - 设置过高：浪费时间
  # 推荐值：3
  retry_max_count: 3
  
  # 单次站立动作的超时时间 (ms)
  # 如果站立超过此时间仍未完成，判定为失败
  # 推荐值：5000-8000 ms
  standup_timeout: 5000
  
  # 重新定位的超时时间 (ms)
  # 定位失败将继续游戏而不阻止程序
  # 推荐值：10000 ms
  reposition_timeout: 10000
  
  # 恢复成功后的延迟 (ms)
  # 站立成功后等待多久再参与游戏
  # 推荐值：500-1000 ms
  post_recovery_delay: 500

# 策略控制参数
strategy:
  # 是否启用自动站立功能
  # false: 禁用自动站立，需要手工干预
  # true: 启用自动站立
  # 建议：产品版本为 true，开发/测试版本可设为 false
  enable_auto_standup: true
  
  # 重试失败时是否自动切换到手动模式
  # true: 恢复失败后自动切换手动控制
  # false: 恢复失败继续自动模式（需要其他逻辑处理）
  # 建议：true (更安全)
  enable_recovery_downgrade: true
  
  # 在比赛暂停期间是否尝试恢复
  # true: 暂停时也尝试站立
  # false: 暂停时不尝试站立
  # 建议：false (避免被罚)
  allow_recovery_during_pause: false
  
  # 定位失败是否阻止游戏继续
  # true: 定位失败则暂停游戏
  # false: 定位失败但继续游戏
  # 建议：false (让游戏继续进行)
  block_on_localization_failure: false

# 摄像头扫描配置（恢复时使用）
camera_recovery:
  # 低位置 pitch（向下看）
  low_pitch: 0.6
  
  # 高位置 pitch（向上看）
  high_pitch: 0.45
  
  # 左向 yaw
  left_yaw: 0.8
  
  # 右向 yaw
  right_yaw: -0.8
  
  # 每个位置停留的时间 (ms)
  msec_per_position: 500
  
  # 完整一个扫描周期的时间 (ms)
  msec_cycle: 3000

# ==================== 调试和日志 ====================

recovery_debug:
  # 是否输出详细日志
  log_detailed_state: true
  
  # 日志输出间隔 (ms)
  log_interval_ms: 200
  
  # 是否使用 rerun 可视化
  use_rerun_logging: true
  
  # 是否保存恢复轨迹
  save_recovery_trajectory: false
  
  # 恢复轨迹保存路径
  trajectory_save_path: "/tmp/recovery_trajectory.csv"

# ==================== 高级参数 ====================

recovery_advanced:
  # 恢复失败后等待多久再尝试 (ms)
  retry_wait_time: 2000
  
  # 定位成功的置信度阈值 (0-1)
  localization_confidence_threshold: 0.6
  
  # 站立后是否需要验证姿态（需要 IMU）
  verify_posture_after_standup: false
  
  # IMU 垂直度检查阈值 (rad)
  # 0.1 rad ≈ 5.7°
  imu_verticality_threshold: 0.1
```

### 1.2 ROS2 参数文件示例 (recovery_config.yaml)

```yaml
/brain_node:
  ros__parameters:
    # 恢复基础配置
    recovery.retry_max_count:
      type: integer
      value: 3
      description: "最大重试次数"
    
    recovery.standup_timeout:
      type: integer
      value: 5000
      description: "站立超时时间 (ms)"
    
    recovery.reposition_timeout:
      type: integer
      value: 10000
      description: "定位超时时间 (ms)"
    
    # 策略控制
    strategy.enable_auto_standup:
      type: boolean
      value: true
      description: "是否启用自动站立"
    
    strategy.enable_recovery_downgrade:
      type: boolean
      value: true
      description: "失败时是否降级到手动模式"
```

---

## 2. 故障排查指南

### 2.1 症状诊断表

| 症状 | 可能原因 | 排查步骤 |
|-----|--------|--------|
| 跌倒后不站起来 | 1. 自动恢复禁用<br>2. 传感器故障<br>3. 站立动作超时 | 1. 检查 `enable_auto_standup`<br>2. 查看 IMU 数据<br>3. 增加 `standup_timeout` |
| 重复跌倒 | 1. 定位失败<br>2. 环境不稳定<br>3. 站立不稳定 | 1. 检查定位日志<br>2. 测试环保<br>3. 查看 IMU 稳定性 |
| 站立成功但无法继续比赛 | 1. 定位失败<br>2. 参数不同步 | 1. 检查定位结果<br>2. 同步配置参数 |
| 重试超限但仍在尝试 | 1. 参数未更新<br>2. 计数器重置 | 1. 检查 `retry_max_count`<br>2. 查看是否有异常重置 |

### 2.2 日志分析方法

#### 2.2.1 正常恢复的日志序列

```
[recovery] State:HAS_FALLEN Mode:3 Retry:0/3 Performed:0 Auto:1
[RECOVERY] Triggered stand up (attempt 1/3)
[recovery] State:IS_GETTING_UP Mode:12 Retry:0/3 Performed:1 Auto:1
[RECOVERY] Stand up attempt completed. Retry count: 1/3
[recovery] State:IS_READY Mode:8 Retry:1/3 Performed:0 Auto:1
[RECOVERY] Recovery completed successfully. Ready to play.
```

#### 2.2.2 失败恢复的日志序列

```
[recovery] State:HAS_FALLEN Mode:3 Retry:0/3 Performed:0 Auto:1
[RECOVERY] Triggered stand up (attempt 1/3)
[recovery] State:HAS_FALLEN Mode:3 Retry:1/3 Performed:0 Auto:1  ← 未进入 IS_GETTING_UP
[RECOVERY] Triggered stand up (attempt 2/3)
[recovery] State:HAS_FALLEN Mode:3 Retry:2/3 Performed:0 Auto:1
[RECOVERY] Triggered stand up (attempt 3/3)
[recovery] State:HAS_FALLEN Mode:3 Retry:3/3 Performed:0 Auto:1
[RECOVERY] WARNING: Recovery failed after 3 attempts. Degrading to manual mode.
```

#### 2.2.3 比赛暂停时的日志

```
[RECOVERY] Game paused/penalty: Reset recovery
[recovery] State:HAS_FALLEN Mode:1 Retry:0/3 Performed:0 Auto:1  ← 模式回到 1
```

### 2.3 常见问题及解决方案

#### 问题 1: 恢复非常慢 (超过 8 秒)

**症状**: 跌倒后需要很长时间才能回到游戏

**排查**:
```bash
# 1. 检查摄像头扫描时间
grep "CamScanField" /var/log/brain.log
# 应该看到约 3000 ms 的周期

# 2. 检查定位时间
grep "Localize" /var/log/brain.log
# 应该看到约 1000-3000 ms

# 3. 查看总耗时
grep "Recovery completed" /var/log/brain.log
```

**解决**:
- 减少 `camera_recovery.msec_cycle`: 从 3000 → 2000
- 使用快速恢复树 (QuickRecovery)
- 增加摄像头帧率

#### 问题 2: 站立成功但定位失败

**症状**: 日志显示恢复完成，但机器人位置错误

**排查**:
```bash
# 查看定位失败的原因
grep "Locate" /var/log/brain.log | grep -i fail

# 检查视觉标志点是否可见
grep "marking" /var/log/brain.log
```

**解决**:
- 检查视觉系统（摄像头清洁）
- 确保球场标志点清晰可见
- 增加 `reposition_timeout`
- 设置 `block_on_localization_failure: false`

#### 问题 3: 重试超过预设次数

**症状**: 尝试超过 3 次仍无法站起来

**排查**:
```bash
# 检查硬件故障
grep "mode" /var/log/motor.log  # 查看电机状态
grep "imu" /var/log/imu.log      # 查看 IMU 数据

# 检查是否有异常重置
grep "recoveryPerformedRetryCount" /var/log/brain.log
```

**解决**:
1. 物理检查：
   - 查看电池电量（<30% 可能导致力量不足）
   - 检查关节是否卡死
   - 检查地面是否平坦
   
2. 参数调整：
   - 增加 `standup_timeout`: 从 5000 → 8000
   - 增加 `retry_max_count`: 从 3 → 5

3. 最后手段：
   - 启用手动模式
   - 或重新启动机器人

---

## 3. 性能调优

### 3.1 快速恢复优化

```yaml
# 为快速恢复调整参数
strategy:
  enable_auto_standup: true

camera_recovery:
  msec_cycle: 1500              # 减少扫描时间
  msec_per_position: 300        # 减少停留时间
  
recovery:
  standup_timeout: 3000         # 减少超时
  reposition_timeout: 5000
```

### 3.2 可靠恢复优化

```yaml
# 为高可靠性调整参数
strategy:
  enable_auto_standup: true
  enable_recovery_downgrade: true  # 失败降级

camera_recovery:
  msec_cycle: 5000              # 增加扫描时间
  msec_per_position: 800        # 增加停留时间
  
recovery:
  standup_timeout: 8000         # 增加超时
  reposition_timeout: 15000
  retry_max_count: 5            # 增加重试
```

### 3.3 低功耗模式

```yaml
# 为电池优化
strategy:
  enable_auto_standup: true

camera_recovery:
  msec_cycle: 2000
  msec_per_position: 400

recovery:
  standup_timeout: 4000
  reposition_timeout: 8000
```

---

## 4. 测试清单

### 4.1 基础功能测试

- [ ] 跌倒检测工作正常
- [ ] 站立命令可以正确发送
- [ ] 重试计数器正确递增
- [ ] 比赛暂停时不站立
- [ ] 恢复失败时记录警告

### 4.2 集成测试

- [ ] 与定位系统集成正常
- [ ] 与摄像头系统集成正常
- [ ] 与游戏控制器集成正常
- [ ] 日志输出正确
- [ ] 语音提示工作正常

### 4.3 压力测试

- [ ] 连续 5 次跌倒 → 恢复成功率 > 90%
- [ ] 在不同位置跌倒 → 都能正确恢复
- [ ] 快速跌倒 (间隔 < 2 秒) → 不重复计数

### 4.4 边界条件测试

- [ ] 在场地边界附近跌倒 → 能回到安全位置
- [ ] 光线不足时 → 定位降级处理
- [ ] 低电量时 (< 20%) → 记录警告
- [ ] 地面不平时 → 多次重试

---

## 5. 监控和告警

### 5.1 关键指标

```python
# 监控脚本示例
import rospy
from std_msgs.msg import Bool, Int32

class RecoveryMonitor:
    def __init__(self):
        self.recovery_failed = False
        self.recovery_count = 0
        self.total_recovery_time = 0
        
    def on_recovery_failed(self, msg):
        self.recovery_failed = msg.data
        if self.recovery_failed:
            rospy.logwarn("ALERT: Robot recovery failed!")
            # 触发告警
            
    def on_recovery_completed(self, msg):
        self.recovery_count += 1
        rospy.loginfo(f"Recovery count: {self.recovery_count}")
```

### 5.2 告警规则

| 告警 | 触发条件 | 严重程度 | 建议动作 |
|-----|--------|--------|--------|
| 恢复失败 | `recovery_failed == true` | 🔴 严重 | 立即切换手动模式 |
| 恢复缓慢 | 恢复耗时 > 10 秒 | 🟡 中等 | 记录日志，查找原因 |
| 重复跌倒 | 1 分钟内 > 3 次 | 🟡 中等 | 检查地面/定位 |
| 定位失败 | 连续定位失败 > 2 次 | 🟢 低 | 监控定位精度 |

---

## 6. 与其他系统的集成

### 6.1 与手动模式的集成

```cpp
// 在 brain.cpp 中
void Brain::tick() {
    // ... 其他逻辑 ...
    
    // 检查恢复失败
    if (tree->getEntry<bool>("recovery_failed")) {
        // 自动切换到手动模式
        if (config->auto_downgrade_on_recovery_failure) {
            tree->setEntry<int>("control_state", 1);  // 手柄模式
            client->setVelocity(0, 0, 0);  // 停止运动
        }
    }
}
```

### 6.2 与语音系统的集成

```cpp
// 恢复相关的语音提示
void Brain::recoverySpeech(const string& message) {
    if (message == "trying_standup") {
        speak("Standing up");  // 或中文："正在站起来"
    } else if (message == "recovery_success") {
        speak("Ready");  // 或中文："准备好了"
    } else if (message == "recovery_failed") {
        speak("Manual mode");  // 或中文："手动模式"
    }
}
```

### 6.3 与日志系统的集成

```cpp
// 输出到 CSV 以便分析
void Brain::logRecoveryStatistics() {
    ofstream log_file("recovery_stats.csv", ios::app);
    log_file << timestamp << ","
             << recovery_count << ","
             << recovery_time_ms << ","
             << localization_result << "\n";
}
```

---

## 7. 版本历史

| 版本 | 日期 | 改动 |
|-----|------|------|
| 1.0 | 2026-01-26 | 初始版本 |

---

**维护者**: AI Assistant  
**联系方式**: [Support Email]  
**最后更新**: 2026-01-26
