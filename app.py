# app_simple.py - 单文件版本，避免导入问题
import streamlit as st
import pandas as pd
import plotly.graph_objects as go
from datetime import datetime
import time
import random

# 心跳模拟器类（直接写在app.py中）
class DroneHeartbeatSimulator:
    def __init__(self):
        self.heartbeats = []
        self.is_connected = True
        self.last_heartbeat_time = None
        self.running = False
        self.simulate_failure = False
        self.failure_start_time = None
        
    def generate_heartbeat(self, seq_num):
        return {
            "序号": seq_num,
            "时间戳": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "完整时间": datetime.now(),
            "延迟_ms": round(random.uniform(5, 50), 2)
        }
    
    def update_heartbeat(self):
        if not self.running:
            return None
        
        if self.simulate_failure:
            if self.failure_start_time is None:
                self.failure_start_time = time.time()
            if time.time() - self.failure_start_time < 4:
                self.check_connection()
                return None
            else:
                self.simulate_failure = False
                self.failure_start_time = None
                self.is_connected = True
        
        seq_num = len(self.heartbeats) + 1
        heartbeat = self.generate_heartbeat(seq_num)
        self.heartbeats.append(heartbeat)
        self.last_heartbeat_time = time.time()
        self.check_connection()
        return heartbeat
    
    def check_connection(self):
        if self.last_heartbeat_time is None:
            return True
        time_since_last = time.time() - self.last_heartbeat_time
        if time_since_last > 3 and self.is_connected:
            self.is_connected = False
        elif time_since_last <= 3 and not self.is_connected:
            self.is_connected = True
        return self.is_connected
    
    def start(self):
        self.running = True
        self.heartbeats = []
        self.last_heartbeat_time = None
        self.is_connected = True
        
    def stop(self):
        self.running = False
    
    def get_heartbeat_data(self):
        return self.heartbeats
    
    def get_connection_status(self):
        self.check_connection()
        return self.is_connected, self.last_heartbeat_time

# Streamlit UI
st.set_page_config(page_title="无人机心跳监测", page_icon="🚁", layout="wide")

# 初始化
if 'simulator' not in st.session_state:
    st.session_state.simulator = DroneHeartbeatSimulator()
    st.session_state.is_monitoring = False

# 侧边栏
with st.sidebar:
    st.title("🎮 控制面板")
    
    if st.button("🚀 启动监测", use_container_width=True):
        st.session_state.simulator.start()
        st.session_state.is_monitoring = True
        st.rerun()
    
    if st.button("⏹️ 停止监测", use_container_width=True):
        st.session_state.simulator.stop()
        st.session_state.is_monitoring = False
        st.rerun()
    
    if st.button("📡 模拟掉线", use_container_width=True):
        if st.session_state.is_monitoring:
            st.session_state.simulator.simulate_failure = True
            st.warning("模拟掉线中...")
    
    total = len(st.session_state.simulator.get_heartbeat_data())
    st.metric("累计心跳包", total)

# 主界面
st.title("🚁 无人机通信心跳监测系统")

# 状态显示
col1, col2, col3 = st.columns(3)
with col1:
    if st.session_state.is_monitoring:
        connected, _ = st.session_state.simulator.get_connection_status()
        if connected:
            st.success("✅ 连接正常")
        else:
            st.error("⚠️ 掉线报警！")
    else:
        st.info("⏸️ 未启动")

# 实时更新
if st.session_state.is_monitoring:
    st.session_state.simulator.update_heartbeat()
    time.sleep(0.5)
    st.rerun()

# 图表
data = st.session_state.simulator.get_heartbeat_data()
if data:
    df = pd.DataFrame(data)
    df['时间'] = pd.to_datetime(df['完整时间'])
    
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df['时间'], y=df['序号'],
        mode='lines+markers',
        name='心跳序号',
        line=dict(color='#667eea', width=2),
        marker=dict(size=6)
    ))
    fig.update_layout(title="心跳序号变化图", height=400)
    st.plotly_chart(fig, use_container_width=True)
    
    st.dataframe(df[['序号', '时间戳', '延迟_ms']].tail(10))
else:
    st.info("点击启动监测开始接收数据")
