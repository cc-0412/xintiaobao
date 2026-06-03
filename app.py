# heartbeat_simulator.py
"""
无人机心跳包模拟程序
每秒发送一个心跳包，包含序号和时间戳
"""

import time
from datetime import datetime
from typing import List, Dict
import random

class DroneHeartbeatSimulator:
    """无人机心跳包模拟器"""
    
    def __init__(self):
        self.heartbeats: List[Dict] = []  # 存储所有心跳包
        self.is_connected = True
        self.last_heartbeat_time = None
        self.running = False
        self.simulate_failure = False  # 模拟掉线开关
        self.failure_start_time = None
        
    def generate_heartbeat(self, seq_num: int) -> Dict:
        """生成单个心跳包"""
        return {
            "序号": seq_num,
            "时间戳": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "完整时间": datetime.now(),
            "状态": "正常",
            "延迟_ms": round(random.uniform(5, 50), 2)  # 模拟网络延迟
        }
    
    def update_heartbeat(self):
        """手动更新一次心跳（用于Streamlit的按钮触发）"""
        if not self.running:
            return None
        
        # 模拟信号中断
        if self.simulate_failure:
            if self.failure_start_time is None:
                self.failure_start_time = time.time()
            
            # 中断持续4秒
            if time.time() - self.failure_start_time < 4:
                # 不生成新心跳，模拟掉线
                self.check_connection()
                return None
            else:
                # 恢复连接
                self.simulate_failure = False
                self.failure_start_time = None
                self.is_connected = True
        
        # 生成新心跳
        seq_num = len(self.heartbeats) + 1
        heartbeat = self.generate_heartbeat(seq_num)
        self.heartbeats.append(heartbeat)
        self.last_heartbeat_time = time.time()
        
        # 检查连接状态
        self.check_connection()
        
        return heartbeat
    
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
    
    def start(self):
        """启动模拟器"""
        self.running = True
        self.heartbeats = []
        self.last_heartbeat_time = None
        self.is_connected = True
        
    def stop(self):
        """停止模拟器"""
        self.running = False
    
    def get_heartbeat_data(self) -> List[Dict]:
        """获取心跳数据列表"""
        return self.heartbeats
    
    def get_latest_heartbeat(self) -> Dict:
        """获取最新心跳数据"""
        if self.heartbeats:
            return self.heartbeats[-1]
        return None
    
    def get_connection_status(self) -> tuple:
        """获取连接状态和最后心跳时间"""
        self.check_connection()
        return self.is_connected, self.last_heartbeat_time


# 独立运行测试
if __name__ == "__main__":
    print("=" * 50)
    print("无人机心跳模拟器测试")
    print("=" * 50)
    
    simulator = DroneHeartbeatSimulator()
    simulator.start()
    
    try:
        for i in range(15):  # 测试15秒
            heartbeat = simulator.update_heartbeat()
            if heartbeat:
                print(f"[心跳] 序号: {heartbeat['序号']}, "
                      f"时间: {heartbeat['时间戳']}, "
                      f"延迟: {heartbeat['延迟_ms']}ms")
            
            status, last_time = simulator.get_connection_status()
            if not status:
                print("⚠️ 当前状态: 掉线中...")
            else:
                print(f"✅ 连接正常")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n模拟器已停止")
        simulator.stop()

