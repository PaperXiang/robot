# 足球机器人跌倒恢复策略 - 快速参考 & 实现清单

## 快速参考卡

### 状态代码速查表

```
IS_READY      = 0  ✓ 机器人就绪
IS_FALLING    = 1  ⬇️  机器人正在跌倒
HAS_FALLEN    = 2  💥 机器人已跌倒 ← [触发站立]
IS_GETTING_UP = 3  ⬆️  机器人正在站起
```

### 模式代码速查表

```
模式  1  : 比赛暂停        → 不执行站立
模式  3  : 已跌倒          → 执行站立
模式  8  : 就绪            → 恢复完成
模式 12  : 正在站立中      → 监听完成信号
```

### 核心变量

| 变量 | 类型 | 含义 |
|-----|------|------|
| `recoveryState` | enum | 当前恢复状态 |
| `currentRobotModeIndex` | int | 当前机器人模式 |
| `recoveryPerformedRetryCount` | int | 已重试次数 |
| `recoveryPerformed` | bool | 本次站立是否已发起 |
| `recovery_failed` | bool | 恢复失败标记 |

### 关键参数

```yaml
recovery.retry_max_count: 3              # 最大重试 3 次
strategy.enable_auto_standup: true       # 启用自动恢复
strategy.enable_recovery_downgrade: true # 失败时降级
```

---

## 实现清单

### ✅ 已完成的实现

- [x] **状态机设计**
  - [x] 4 种状态定义 (IS_READY, IS_FALLING, HAS_FALLEN, IS_GETTING_UP)
  - [x] 状态转移条件清晰
  - [x] 状态转移图完成

- [x] **CheckAndStandUp 节点增强**
  - [x] 詳細日志输出 (5 种日志级别)
  - [x] 完整的注释和文档
  - [x] 重试计数逻辑
  - [x] 暂停时重置逻辑
  - [x] 失败时设置标记
  - [x] 恢复完成后重置

- [x] **行为树集成**
  - [x] 基础恢复树 (AutoGetUpAndLocate)
  - [x] 高级恢复树 (AdvancedRecovery - 可选)
  - [x] 快速恢复树 (QuickRecovery - 轻量级)
  - [x] 集成定位系统
  - [x] 处理比赛暂停情况

- [x] **文档完成**
  - [x] 策略设计文档 (RECOVERY_STRATEGY_GUIDE.md)
  - [x] 故障排查指南 (RECOVERY_TROUBLESHOOTING.md)
  - [x] 配置示例和说明
  - [x] 日志分析方法
  - [x] 性能调优建议
  - [x] 常见问题解答

- [x] **测试工具**
  - [x] 单元测试 (4 个)
  - [x] 集成测试 (2 个)
  - [x] 性能测试 (1 个)
  - [x] 压力测试 (2 个)
  - [x] Python 测试脚本

### 🔄 可选/后续实现

- [ ] **IMU 传感器集成**
  - [ ] 垂直度检查
  - [ ] 跌倒预测
  - [ ] 位置验证

- [ ] **高级功能**
  - [ ] 多种站立策略选择
  - [ ] 自适应定位
  - [ ] 机器学习优化

- [ ] **监控和告警**
  - [ ] 实时监控 Dashboard
  - [ ] 告警系统集成
  - [ ] 统计数据收集

- [ ] **改进项**
  - [ ] 支持更多机器人型号
  - [ ] 支持动态参数调整
  - [ ] 集成更多传感器

---

## 文件结构

```
src/brain/
├── behavior_trees/
│   ├── game.xml                          # 主游戏树（已包含恢复树）
│   └── subtrees/
│       ├── subtree_auto_standup_and_locate.xml  # 基础恢复树 ✓
│       ├── subtree_recovery_strategy.xml        # 增强恢复树 ✓ (新建)
│       └── ... (其他树)
│
├── src/
│   └── brain_tree.cpp                    # CheckAndStandUp 实现增强 ✓
│
├── include/
│   ├── brain_tree.h                      # CheckAndStandUp 声明
│   └── types.h                           # RobotRecoveryState 定义
│
├── config/
│   └── config.yaml                       # 配置参数 (需更新)
│
├── RECOVERY_STRATEGY_GUIDE.md            # 策略文档 ✓ (新建)
├── RECOVERY_TROUBLESHOOTING.md           # 故障排查 ✓ (新建)
└── test_recovery_strategy.py             # 测试脚本 ✓ (新建)
```

---

## 快速开始指南

### 第 1 步：验证文件已创建

```bash
# 检查新创建的文件
ls -l src/brain/RECOVERY_STRATEGY_GUIDE.md
ls -l src/brain/RECOVERY_TROUBLESHOOTING.md
ls -l src/brain/test_recovery_strategy.py
ls -l src/brain/behavior_trees/subtrees/subtree_recovery_strategy.xml
```

### 第 2 步：编译代码

```bash
cd src/brain
mkdir -p build
cd build
cmake ..
make -j4
```

### 第 3 步：运行测试

```bash
# 单元测试
python3 test_recovery_strategy.py --mode unit

# 集成测试
python3 test_recovery_strategy.py --mode integration

# 压力测试 (10 次迭代)
python3 test_recovery_strategy.py --mode stress --iterations 10

# 全部测试
python3 test_recovery_strategy.py --mode all
```

### 第 4 步：配置参数

在 `config/config.yaml` 中添加：

```yaml
recovery:
  retry_max_count: 3

strategy:
  enable_auto_standup: true
  enable_recovery_downgrade: true
```

### 第 5 步：启动机器人

```bash
# 启动 brain 节点
ros2 run brain brain_node

# 在另一个终端监控恢复过程
ros2 topic echo /brain_node/recovery
```

---

## 常用命令速查

```bash
# 查看恢复日志
tail -f ~/.ros/log/brain_node.log | grep -i recovery

# 查看恢复统计
python3 << 'EOF'
import json
with open('recovery_test_results.json') as f:
    results = json.load(f)
    passed = sum(1 for r in results if r['passed'])
    print(f"Passed: {passed}/{len(results)}")
EOF

# 实时监控恢复状态
watch -n 1 'ros2 topic echo --once /brain_node/recovery_status'

# 清除恢复失败标记
ros2 service call /brain_node/clear_recovery_failed std_srvs/srv/Empty

# 强制禁用自动恢复
ros2 param set /brain_node recovery.retry_max_count 0
```

---

## 调试技巧

### 启用详细日志

```python
# 在 brain.cpp 中
brain->log->setLogLevel("recovery", "DEBUG");
```

### 观察状态变化

```bash
# 实时打印状态
ros2 param get /brain_node /recovery_state
ros2 param get /brain_node /current_robot_mode_index
ros2 param get /brain_node /recovery_retry_count
```

### 模拟跌倒

```python
# 通过 ROS 服务模拟
ros2 service call /brain_node/simulate_fall std_srvs/srv/Empty
```

---

## 性能指标参考

### 理想恢复时间

- **快速恢复**: 3.5-4.5 秒 (QuickRecovery 树)
- **标准恢复**: 5.0-7.0 秒 (AutoGetUpAndLocate 树)
- **完整恢复**: 7.0-10.0 秒 (AdvancedRecovery 树)

### 成功率目标

- **第 1 次**: ≥ 75%
- **第 2 次**: ≥ 85%
- **第 3 次**: ≥ 90%
- **总体**: ≥ 95%

---

## 问题排查决策树

```
❓ 机器人不站起来
├─ 检查日志是否有 "Triggered stand up"?
│  ├─ 无 → enable_auto_standup 可能为 false
│  └─ 有 → 继续
├─ 检查 recoveryState 是否为 HAS_FALLEN?
│  ├─ 否 → IMU 可能故障
│  └─ 是 → 继续
├─ 检查电池电量?
│  ├─ < 20% → 更换电池
│  └─ ≥ 20% → 继续
└─ 联系技术支持

❓ 恢复太慢
├─ 使用 QuickRecovery 树 (减少定位时间)
├─ 增加摄像头帧率
└─ 检查定位耗时是否过长

❓ 重复失败
├─ 增加 retry_max_count (从 3 → 5)
├─ 增加 standup_timeout (从 5000 → 8000)
└─ 检查地面是否平坦
```

---

## 与其他系统的接口

### 与定位系统

```python
# 恢复后触发定位
if recovery_completed:
    call_service('localize')
```

### 与游戏控制器

```python
# 检查比赛状态
if game_state in ['TIMEOUT', 'INITIAL']:
    inhibit_recovery()
```

### 与手动模式

```python
# 失败时切换
if recovery_failed:
    switch_to_manual_mode()
```

---

## 验证清单 (部署前)

- [ ] 所有新文件已创建
- [ ] 代码已编译且无错误
- [ ] 单元测试通过
- [ ] 集成测试通过
- [ ] 压力测试通过 (≥ 90% 成功率)
- [ ] 配置参数已设置
- [ ] 日志能正确输出
- [ ] 语音提示能正常播放
- [ ] 与定位系统集成正常
- [ ] 与游戏控制器集成正常
- [ ] 在实际机器人上测试过
- [ ] 文档已评审
- [ ] 性能指标符合预期

---

## 支持和反馈

### 遇到问题?

1. 查看 [RECOVERY_TROUBLESHOOTING.md](RECOVERY_TROUBLESHOOTING.md)
2. 检查日志输出 (rerun "recovery" 频道)
3. 运行测试脚本诊断
4. 查阅这个快速参考卡

### 想改进?

1. 查看 [RECOVERY_STRATEGY_GUIDE.md](RECOVERY_STRATEGY_GUIDE.md) 的"扩展和改进"部分
2. 提交改进建议
3. 贡献代码和测试用例

---

**最后更新**: 2026-01-26  
**版本**: 1.0  
**维护者**: AI Assistant
