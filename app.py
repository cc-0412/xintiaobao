
# heartbeat_simulator.py
"""
无人机心跳包模拟程序
每秒发送一个心跳包，包含序号和时间戳
"""

import time
import random
import threading
from datetime import datetime
from typing import List, Dict, Optional

class DroneHeartbeatSimulator:
    """无人机心跳包模拟器"""
    
    def __init__(self):
        self.heartbeats: List[Dict] = []  # 存储所有心跳包
        self.is_connected = True
        self.last_heartbeat_time = None
        self.running = False
        self.simulate_failure = False  # 模拟掉线开关
        
    def generate_heartbeat(self, seq_num: int) -> Dict:
        """生成单个心跳包"""
        return {
            "序号": seq_num,
            "时间戳": datetime.now().strftime("%H:%M:%S"),
            "原始时间": datetime.now(),
            "状态": "正常"
        }
    
    def start_simulator(self, callback=None):
        """启动心跳模拟器"""
        self.running = True
        seq = 0
        
        while self.running:
            try:
                seq += 1
                heartbeat = self.generate_heartbeat(seq)
                self.heartbeats.append(heartbeat)
                self.last_heartbeat_time = time.time()
                
                # 调用回调函数（用于Streamlit实时更新）
                if callback:
                    callback(heartbeat)
                
                print(f"[心跳] 序号: {seq}, 时间: {heartbeat['时间戳']}")
                
                # 模拟随机掉线（可选）
                if self.simulate_failure and random.random() < 0.1:
                    print("[警告] 模拟信号中断...")
                    time.sleep(4)  # 模拟掉线4秒
                else:
                    time.sleep(1)
                    
            except Exception as e:
                print(f"心跳生成错误: {e}")
                time.sleep(1)
    
    def check_connection(self) -> bool:
        """检查连接状态，3秒没收到心跳判定为掉线"""
        if self.last_heartbeat_time is None:
            return True
        
        time_since_last = time.time() - self.last_heartbeat_time
        
        if time_since_last > 3 and self.is_connected:
            self.is_connected = False
            print(f"[掉线报警] 已{time_since_last:.1f}秒未收到心跳包!")
        elif time_since_last <= 3 and not self.is_connected:
            self.is_connected = True
            print("[恢复] 心跳连接已恢复!")
        
        return self.is_connected
    
    def stop_simulator(self):
        """停止模拟器"""
        self.running = False
    
    def get_heartbeat_data(self) -> List[Dict]:
        """获取心跳数据列表"""
        return self.heartbeats


# 独立运行测试
if __name__ == "__main__":
    print("=" * 50)
    print("无人机心跳模拟器测试")
    print("=" * 50)
    
    simulator = DroneHeartbeatSimulator()
    
    # 在后台线程中运行
    def test_callback(hb):
        # 检查连接状态
        simulator.check_connection()
    
    try:
        simulator.start_simulator(callback=test_callback)
    except KeyboardInterrupt:
        print("\n模拟器已停止")
        simulator.stop_simulator()
