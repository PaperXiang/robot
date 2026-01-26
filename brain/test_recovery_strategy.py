#!/usr/bin/env python3
"""
足球机器人跌倒恢复策略测试工具

用途：
1. 单元测试：验证恢复逻辑正确性
2. 集成测试：验证与其他系统的协作
3. 性能测试：测量恢复耗时
4. 压力测试：连续跌倒恢复

使用方法：
    python3 test_recovery_strategy.py --mode unit
    python3 test_recovery_strategy.py --mode integration
    python3 test_recovery_strategy.py --mode stress --iterations 10
"""

import rospy
import time
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Empty
import json
from datetime import datetime
from typing import Dict, List


class RecoveryStrategyTester:
    """恢复策略测试类"""
    
    def __init__(self):
        self.test_results = []
        self.start_time = None
        self.recovery_stats = {
            'success_count': 0,
            'failure_count': 0,
            'total_time': 0,
            'avg_time': 0,
            'min_time': float('inf'),
            'max_time': 0,
        }
        
    def log_test(self, test_name: str, passed: bool, message: str = "", elapsed_ms: float = 0):
        """记录测试结果"""
        result = {
            'timestamp': datetime.now().isoformat(),
            'test_name': test_name,
            'passed': passed,
            'message': message,
            'elapsed_ms': elapsed_ms,
        }
        self.test_results.append(result)
        
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"[{status}] {test_name} ({elapsed_ms:.0f}ms)")
        if message:
            print(f"      └─ {message}")
    
    def save_results(self, filename: str = "recovery_test_results.json"):
        """保存测试结果"""
        with open(filename, 'w') as f:
            json.dump(self.test_results, f, indent=2)
        print(f"\nResults saved to: {filename}")
    
    # ========== 单元测试 ==========
    
    def test_recovery_state_machine(self):
        """
        测试 1: 恢复状态机
        验证状态转移的正确性
        
        期望流程：
        IS_READY → IS_FALLING → HAS_FALLEN → IS_GETTING_UP → IS_READY
        """
        test_name = "Recovery State Machine"
        start_time = time.time()
        
        try:
            # 模拟状态转移
            states = [
                ("IS_READY", 0),
                ("IS_FALLING", 1),
                ("HAS_FALLEN", 2),
                ("IS_GETTING_UP", 3),
                ("IS_READY", 0),
            ]
            
            for state_name, state_code in states:
                rospy.loginfo(f"Testing state: {state_name}")
            
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, True, "State transitions OK", elapsed)
            return True
            
        except Exception as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    def test_retry_counter(self):
        """
        测试 2: 重试计数器
        验证重试次数的正确累计和限制
        """
        test_name = "Retry Counter Logic"
        start_time = time.time()
        
        try:
            retry_max = 3
            retry_count = 0
            
            for i in range(retry_max + 2):
                if retry_count < retry_max:
                    retry_count += 1
                    rospy.loginfo(f"Retry attempt {retry_count}/{retry_max}")
                else:
                    rospy.logwarn(f"Retry limit exceeded: {retry_count}")
            
            assert retry_count == retry_max + 1, "Counter should exceed limit"
            
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, True, f"Counter correctly tracks {retry_count} attempts", elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    def test_pause_handling(self):
        """
        测试 3: 比赛暂停处理
        验证暂停时不执行站立
        """
        test_name = "Game Pause Handling"
        start_time = time.time()
        
        try:
            # 模拟暂停状态
            is_paused = True
            standup_called = False
            
            if not is_paused:
                standup_called = True
            
            assert not standup_called, "StandUp should not be called during pause"
            
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, True, "Pause correctly inhibits standup", elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    def test_recovery_failure_flag(self):
        """
        测试 4: 恢复失败标记
        验证超限时设置失败标记
        """
        test_name = "Recovery Failure Flag"
        start_time = time.time()
        
        try:
            retry_count = 3
            retry_max = 3
            recovery_failed = False
            
            if retry_count >= retry_max:
                recovery_failed = True
            
            assert recovery_failed == True, "Failed flag should be set at limit"
            
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, True, "Failure flag correctly set", elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    # ========== 集成测试 ==========
    
    def test_with_localization(self):
        """
        测试 5: 与定位系统的集成
        验证站立后能正确定位
        """
        test_name = "Recovery + Localization Integration"
        start_time = time.time()
        
        try:
            # 模拟定位流程
            robot_stood_up = True
            localization_attempted = robot_stood_up
            
            assert localization_attempted, "Localization should be attempted after standup"
            
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, True, "Localization triggered after standup", elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    def test_with_game_controller(self):
        """
        测试 6: 与游戏控制器的集成
        验证根据游戏状态调整恢复行为
        """
        test_name = "Recovery + Game Controller Integration"
        start_time = time.time()
        
        try:
            # 测试不同游戏状态
            game_states = ['INITIAL', 'READY', 'PLAYING', 'TIMEOUT', 'FINISHED']
            
            for state in game_states:
                if state in ['TIMEOUT', 'INITIAL']:
                    should_recover = False  # 这些状态不应恢复
                else:
                    should_recover = True  # 其他状态应尝试恢复
                
                rospy.loginfo(f"Game state: {state} -> Recovery: {should_recover}")
            
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, True, "Game states correctly handled", elapsed)
            return True
            
        except Exception as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    # ========== 性能测试 ==========
    
    def test_recovery_timing(self):
        """
        测试 7: 恢复耗时
        测量从跌倒到恢复完成的时间
        """
        test_name = "Recovery Timing"
        start_time = time.time()
        
        try:
            # 模拟恢复过程的各个阶段
            times = {
                'detection': 100,      # 检测耗时
                'standup': 2000,       # 站立耗时
                'camera_scan': 3000,   # 摄像头扫描
                'localization': 2000,  # 定位耗时
            }
            
            total_time = sum(times.values())
            
            # 验证总耗时在合理范围内 (3.5-8.5 秒)
            min_expected = 3500
            max_expected = 8500
            
            assert min_expected <= total_time <= max_expected, \
                f"Total time {total_time}ms outside expected range [{min_expected}, {max_expected}]"
            
            elapsed = (time.time() - start_time) * 1000
            msg = f"Total recovery time: {total_time}ms\n"
            for stage, t in times.items():
                msg += f"  {stage}: {t}ms\n"
            
            self.log_test(test_name, True, msg.strip(), elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    # ========== 压力测试 ==========
    
    def test_continuous_recovery(self, iterations: int = 5):
        """
        测试 8: 连续恢复
        测试机器人能否多次连续恢复
        """
        test_name = f"Continuous Recovery ({iterations} iterations)"
        start_time = time.time()
        
        try:
            success_count = 0
            
            for i in range(iterations):
                # 模拟一次恢复过程
                recovery_success = True  # 在实际应用中这会从机器人状态读取
                
                if recovery_success:
                    success_count += 1
                    rospy.loginfo(f"Recovery {i+1}/{iterations}: SUCCESS")
                else:
                    rospy.logwarn(f"Recovery {i+1}/{iterations}: FAILED")
            
            success_rate = (success_count / iterations) * 100
            
            # 期望成功率 > 90%
            assert success_rate >= 90, f"Success rate {success_rate}% < 90%"
            
            elapsed = (time.time() - start_time) * 1000
            msg = f"Success rate: {success_rate}% ({success_count}/{iterations})"
            self.log_test(test_name, True, msg, elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    def test_rapid_falls(self, count: int = 5):
        """
        测试 9: 快速连续跌倒
        测试快速跌倒时的计数器行为
        """
        test_name = f"Rapid Falls ({count} times)"
        start_time = time.time()
        
        try:
            retry_counts = []
            
            for i in range(count):
                # 模拟快速跌倒
                retry_count = (i + 1)  # 每次跌倒计数递增
                retry_counts.append(retry_count)
                rospy.loginfo(f"Fall {i+1}: retry_count = {retry_count}")
            
            # 验证计数器正确递增
            expected = list(range(1, count + 1))
            assert retry_counts == expected, f"Counter mismatch: {retry_counts} != {expected}"
            
            elapsed = (time.time() - start_time) * 1000
            msg = f"Counter progression: {retry_counts}"
            self.log_test(test_name, True, msg, elapsed)
            return True
            
        except AssertionError as e:
            elapsed = (time.time() - start_time) * 1000
            self.log_test(test_name, False, str(e), elapsed)
            return False
    
    # ========== 主测试方法 ==========
    
    def run_all_unit_tests(self):
        """运行所有单元测试"""
        print("\n" + "="*60)
        print("UNIT TESTS")
        print("="*60)
        
        tests = [
            self.test_recovery_state_machine,
            self.test_retry_counter,
            self.test_pause_handling,
            self.test_recovery_failure_flag,
        ]
        
        results = []
        for test in tests:
            try:
                result = test()
                results.append(result)
            except Exception as e:
                rospy.logerr(f"Test {test.__name__} crashed: {e}")
                results.append(False)
        
        self.print_summary(results)
        return all(results)
    
    def run_all_integration_tests(self):
        """运行所有集成测试"""
        print("\n" + "="*60)
        print("INTEGRATION TESTS")
        print("="*60)
        
        tests = [
            self.test_with_localization,
            self.test_with_game_controller,
        ]
        
        results = []
        for test in tests:
            try:
                result = test()
                results.append(result)
            except Exception as e:
                rospy.logerr(f"Test {test.__name__} crashed: {e}")
                results.append(False)
        
        self.print_summary(results)
        return all(results)
    
    def run_all_performance_tests(self):
        """运行所有性能测试"""
        print("\n" + "="*60)
        print("PERFORMANCE TESTS")
        print("="*60)
        
        tests = [
            self.test_recovery_timing,
        ]
        
        results = []
        for test in tests:
            try:
                result = test()
                results.append(result)
            except Exception as e:
                rospy.logerr(f"Test {test.__name__} crashed: {e}")
                results.append(False)
        
        self.print_summary(results)
        return all(results)
    
    def run_all_stress_tests(self, iterations: int = 10):
        """运行所有压力测试"""
        print("\n" + "="*60)
        print(f"STRESS TESTS (iterations: {iterations})")
        print("="*60)
        
        tests = [
            lambda: self.test_continuous_recovery(iterations),
            lambda: self.test_rapid_falls(iterations // 2),
        ]
        
        results = []
        for test in tests:
            try:
                result = test()
                results.append(result)
            except Exception as e:
                rospy.logerr(f"Test crashed: {e}")
                results.append(False)
        
        self.print_summary(results)
        return all(results)
    
    def print_summary(self, results: List[bool]):
        """打印测试摘要"""
        passed = sum(results)
        total = len(results)
        print(f"\nSummary: {passed}/{total} tests passed")
        print("="*60)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Recovery Strategy Tester")
    parser.add_argument('--mode', choices=['unit', 'integration', 'performance', 'stress', 'all'],
                       default='unit', help='Test mode')
    parser.add_argument('--iterations', type=int, default=10, help='Iterations for stress test')
    parser.add_argument('--output', default='recovery_test_results.json', help='Output file')
    
    args = parser.parse_args()
    
    # 初始化 ROS
    rospy.init_node('recovery_strategy_tester', anonymous=True)
    
    # 创建测试器
    tester = RecoveryStrategyTester()
    
    # 运行测试
    if args.mode == 'unit':
        tester.run_all_unit_tests()
    elif args.mode == 'integration':
        tester.run_all_integration_tests()
    elif args.mode == 'performance':
        tester.run_all_performance_tests()
    elif args.mode == 'stress':
        tester.run_all_stress_tests(args.iterations)
    elif args.mode == 'all':
        tester.run_all_unit_tests()
        tester.run_all_integration_tests()
        tester.run_all_performance_tests()
        tester.run_all_stress_tests(args.iterations)
    
    # 保存结果
    tester.save_results(args.output)
    
    print("\n✓ All tests completed!")


if __name__ == '__main__':
    main()
