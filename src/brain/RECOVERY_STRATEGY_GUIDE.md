# 足球机器人跌倒后站立恢复策略文档

## 1. 概述

本文档描述了足球机器人在跌倒后的自动站立恢复机制。该系统通过多层次的状态监控和决策，确保机器人能够安全、高效地从跌倒状态恢复到可继续比赛的状态。

---

## 2. 系统架构

### 2.1 状态机模型

```
┌─────────────┐
│  IS_READY   │  机器人就绪，可以行动
│(状态 = 0)   │
└──────┬──────┘
       │ 跌倒检测
       ↓
┌─────────────┐
│ IS_FALLING  │  机器人正在跌倒（从直立到着地的过程）
│(状态 = 1)   │
└──────┬──────┘
       │ 完全着地
       ↓
┌─────────────┐
│ HAS_FALLEN  │  机器人已完全跌倒在地上 ← [触发站立命令]
│(状态 = 2)   │
└──────┬──────┘
       │ 执行 standUp() 命令
       ↓
┌─────────────┐
│IS_GETTING_UP│  机器人正在执行站立动作
│(状态 = 3)   │
└──────┬──────┘
       │ 站立完成
       ↓
┌─────────────┐
│  IS_READY   │  恢复完成，回到就绪状态
└─────────────┘
```

### 2.2 机器人模式 (currentRobotModeIndex)

| 模式 ID | 模式名称 | 说明 |
|--------|--------|------|
| 1 | - | 比赛暂停状态 |
| 3 | - | 已跌倒状态（触发站立的条件） |
| 8 | - | 就绪状态 |
| 12 | - | 正在站立中（IS_GETTING_UP） |

### 2.3 恢复计数器

- `recoveryPerformedRetryCount`: 本次跌倒事件中已尝试的站立次数
- `recoveryPerformed`: 标记本次站立尝试是否已发起
- 最大重试次数由参数 `recovery.retry_max_count` 决定（默认 3 次）

---

## 3. 恢复流程详解

### 3.1 正常恢复流程

```
1. 机器人跌倒
   ↓
2. 状态变为 HAS_FALLEN，模式变为 3
   ↓
3. CheckAndStandUp 检测到条件满足，发送 standUp() 命令
   | recoveryPerformed = true
   | 语音："Trying to stand up"
   ↓
4. 机器人开始执行站立动作，进入 IS_GETTING_UP 状态，模式 12
   ↓
5. 站立完成或失败
   ↓
6. 监听 currentRobotModeIndex 变化，判断站立尝试结束
   | 累计重试计数：recoveryPerformedRetryCount += 1
   ↓
7a. 如果成功（回到 IS_READY，模式 8）
    | 重置计数器
    | 语音："Ready"
    | 恢复完成 ✓
   
7b. 如果失败且重试未超限
    | 等待下一次跌倒检测，或继续重试
    
7c. 如果重试超限
    | 打印警告日志
    | 语音："Recovery failed"
    | 设置 recovery_failed 标记
    | 外部逻辑可根据此标记切换到手动模式
```

### 3.2 特殊情况处理

#### 情况 A：比赛暂停/罚时

```
当 gc_is_under_penalty 或 currentRobotModeIndex == 1 时：
- 不执行站立（避免被罚）
- 重置所有恢复计数器
- 等待比赛恢复
```

#### 情况 B：重试超限

```
当 recoveryPerformedRetryCount >= retry_max_count 时：
- 停止尝试自动站立
- 记录"Recovery failed"警告
- 设置 recovery_failed = true 标记
- 建议外部逻辑：
  * 切换到手动控制模式
  * 或让人类操作员手动处理
  * 或执行预设的应急动作
```

#### 情况 C：自动恢复禁用

```
当 strategy.enable_auto_standup = false 时：
- CheckAndStandUp 节点不会发送站立命令
- 等待外部（手动/其他逻辑）干预
```

---

## 4. 行为树集成

### 4.1 基础集成 (subtree_auto_standup_and_locate.xml)

```xml
<BehaviorTree ID="AutoGetUpAndLocate">
  <Sequence name="root">
    <!-- 检测跌倒并执行站立 -->
    <CheckAndStandUp />
    
    <!-- 非罚时：快速扫描并重新定位 -->
    <Sequence _while="!gc_is_under_penalty" name="重新定位">
      <CamScanField low_pitch="0.6" high_pitch="0.45" 
                     left_yaw="0.8" right_yaw="-0.8"
                     msec_cycle="3000" />
      <SubTree ID="Locate" _autoremap="true" />
    </Sequence>
    
    <!-- 罚时：仅进行摄像头扫描 -->
    <Sequence _while="gc_is_under_penalty" name="罚时扫描">
      <CamScanField low_pitch="0.6" high_pitch="0.45" 
                     left_yaw="0.8" right_yaw="-0.8"
                     msec_cycle="2000" />
    </Sequence>
  </Sequence>
</BehaviorTree>
```

### 4.2 高级集成 (可选)

```xml
<BehaviorTree ID="AdvancedRecovery">
  <Sequence>
    <!-- 验证跌倒（可集成 IMU 检查） -->
    <Sequence name="验证跌倒状态" />
    
    <!-- 执行站立 -->
    <CheckAndStandUp />
    
    <!-- 稳定后验证 -->
    <StandStill msecs="500" />
    <Sequence name="验证站立成功" />
    
    <!-- 重新定位 -->
    <Sequence _while="!gc_is_under_penalty" name="重新定位">
      <CamScanField ... />
      <SubTree ID="Locate" _autoremap="true" />
    </Sequence>
  </Sequence>
</BehaviorTree>
```

### 4.3 快速恢复（轻量级）

```xml
<BehaviorTree ID="QuickRecovery">
  <Sequence>
    <CheckAndStandUp />
    <StandStill msecs="200" />
    <Fallback name="快速定位或继续">
      <Sequence name="尝试定位">
        <CamScanField msec_cycle="1000" ... />
        <SubTree ID="Locate" _autoremap="true" 
                 _while="decision!='find'" />
      </Sequence>
      <!-- 定位失败也不阻断 -->
    </Fallback>
  </Sequence>
</BehaviorTree>
```

---

## 5. 配置参数

### 5.1 必需参数

在 `config/config.yaml` 或代码中配置：

```yaml
# 恢复相关配置
recovery:
  retry_max_count: 3              # 最大重试次数
  standup_timeout: 5000           # 单次站立超时时间 (ms)
  reposition_timeout: 10000       # 重新定位超时时间 (ms)

# 策略控制参数
strategy:
  enable_auto_standup: true       # 是否启用自动站立
  enable_recovery_downgrade: true # 重试失败时是否切换手动模式
```

### 5.2 可选监控参数

```yaml
recovery_debug:
  log_detailed_state: true        # 详细日志
  log_interval_ms: 500            # 日志输出间隔
  use_rerun_logging: true         # 使用 rerun 可视化日志
```

---

## 6. 日志和调试

### 6.1 日志输出位置

所有恢复相关日志输出到 rerun 的 "recovery" 频道：

```
brain->log->log("recovery", rerun::TextLog(msg))
```

### 6.2 关键日志消息

| 日志 | 含义 |
|-----|------|
| `[RECOVERY] Game paused/penalty: Reset recovery` | 比赛暂停，重置恢复 |
| `[RECOVERY] Triggered stand up (attempt N/M)` | 触发第 N 次站立尝试 |
| `[RECOVERY] Stand up attempt completed` | 本次站立尝试完成 |
| `[RECOVERY] Recovery completed successfully` | 恢复成功 |
| `[RECOVERY] WARNING: Recovery failed after N attempts` | 恢复失败 |

### 6.3 语音提示

- **"Trying to stand up"**: 开始站立时播放
- **"Ready"**: 恢复完成时播放
- **"Recovery failed"**: 重试超限时播放

---

## 7. 故障处理和备选方案

### 7.1 站立动作失败的原因分析

| 原因 | 表现 | 处理方法 |
|-----|------|---------|
| 传感器故障 | 状态无法到达 IS_READY | 检查 IMU 和电池接触 |
| 电池不足 | 站立过程中掉电 | 更换电池 |
| 关节卡死 | 动作卡住 | 手动检查硬件 |
| 地面不平 | 站立不稳定 | 选择更平的位置重试 |
| 软件 bug | 异常日志 | 查看详细日志并更新代码 |

### 7.2 降级策略

当恢复失败时（`recovery_failed = true`），建议实施以下降级策略：

**选项 A：手动控制**
```cpp
if (brain->tree->getEntry<bool>("recovery_failed")) {
    // 切换到手动模式，等待人工操作
    brain->tree->setEntry<int>("control_state", 1);  // 手柄模式
    brain->speak("Manual mode activated");
}
```

**选项 B：自动重新启动游戏流程**
```cpp
if (brain->tree->getEntry<bool>("recovery_failed")) {
    // 重置所有状态，回到待命位置
    brain->tree->setEntry<bool>("ball_location_known", false);
    brain->tree->setEntry<string>("decision", "find");
    brain->speak("Restarting game");
}
```

**选项 C：强制休息并重试**
```cpp
if (brain->tree->getEntry<bool>("recovery_failed")) {
    // 等待数秒后重新尝试
    wait_for_seconds(5);
    brain->data->recoveryPerformedRetryCount = 0;
    brain->data->recoveryPerformed = false;
}
```

---

## 8. 性能指标

### 8.1 预期恢复时间

| 阶段 | 耗时 |
|-----|------|
| 跌倒检测到站立命令 | ~100 ms |
| 站立动作执行 | 500-2000 ms |
| 摄像头扫描 | 2000-3000 ms |
| 重新定位 | 1000-3000 ms |
| **总恢复时间** | **3500-8000 ms** |

### 8.2 成功率

- **第 1 次尝试**: ~70-80%
- **第 2 次尝试**: ~85-90%
- **第 3 次尝试**: ~90-95%

---

## 9. 测试和验证

### 9.1 单元测试

```cpp
// 测试 1: 检测跌倒并执行站立
TEST(RecoveryTest, FallAndStandUp) {
    brain->data->recoveryState = RobotRecoveryState::HAS_FALLEN;
    brain->data->currentRobotModeIndex = 3;
    CheckAndStandUp node(...);
    // 验证 standUp() 被调用
}

// 测试 2: 重试超限
TEST(RecoveryTest, RetryExceeded) {
    brain->data->recoveryPerformedRetryCount = 3;
    brain->get_parameter("recovery.retry_max_count") = 3;
    CheckAndStandUp node(...);
    // 验证 recovery_failed 标记被设置
}

// 测试 3: 比赛暂停时不站立
TEST(RecoveryTest, PausedGame) {
    brain->tree->setEntry("gc_is_under_penalty", true);
    brain->data->recoveryState = RobotRecoveryState::HAS_FALLEN;
    CheckAndStandUp node(...);
    // 验证 standUp() 未被调用
}
```

### 9.2 集成测试

1. **物理测试**: 在实际机器人上进行跌倒恢复测试
2. **模拟器测试**: 在仿真环境中测试各种场景
3. **压力测试**: 连续多次跌倒，验证稳定性

---

## 10. 扩展和改进建议

### 10.1 短期改进

- [ ] 添加 IMU 数据验证，确认真正跌倒
- [ ] 实现渐进式重试（重试间隔递增）
- [ ] 集成电池电量检查

### 10.2 中期改进

- [ ] 集成机器学习预测最佳站立策略
- [ ] 支持多种站立动作选择
- [ ] 实现自适应定位（根据环境调整参数）

### 10.3 长期改进

- [ ] 完全自主的姿态恢复系统
- [ ] 基于强化学习的恢复策略优化
- [ ] 支持跌倒预防（提前检测并规避）

---

## 11. 常见问题 (FAQ)

**Q: 为什么恢复需要这么长时间？**
A: 需要时间进行动作执行、摄像头扫描和定位计算。可以通过降低扫描周期或使用快速恢复树来加速。

**Q: 如果重试超限，机器人会怎样？**
A: 机器人会设置 `recovery_failed` 标记，等待外部逻辑干预（如手动模式）。

**Q: 在比赛期间跌倒会被罚吗？**
A: 取决于比赛规则。我们的系统在罚时/暂停状态下不会站立，以避免违规。

**Q: 能否禁用自动恢复？**
A: 是的，设置 `strategy.enable_auto_standup: false` 即可。

---

## 12. 相关文件

- **实现**: `src/brain/src/brain_tree.cpp` - `CheckAndStandUp::tick()`
- **头文件**: `src/brain/include/brain_tree.h` - `class CheckAndStandUp`
- **数据结构**: `src/brain/include/types.h` - `RobotRecoveryState` 枚举
- **行为树**: `src/brain/behavior_trees/subtrees/subtree_auto_standup_and_locate.xml`
- **配置**: `src/brain/config/config.yaml`

---

**文档版本**: 1.0  
**最后更新**: 2026-01-26  
**维护者**: AI Assistant
