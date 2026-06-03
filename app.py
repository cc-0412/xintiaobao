"""
无人机任务规划与监控系统
包含航线规划（3D地图）和飞行监控（心跳监测）
"""

import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from datetime import datetime
import time
import random
import math

# ==================== 坐标系转换工具 ====================
def gcj02_to_wgs84(lng, lat):
    """GCJ-02 转 WGS-84"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    
    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def wgs84_to_gcj02(lng, lat):
    """WGS-84 转 GCJ-02"""
    a = 6378245.0
    ee = 0.00669342162296594323
    
    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    
    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    return lng + dlng, lat + dlat

def calculate_distance(lat1, lng1, lat2, lng2):
    """计算两点距离（米）"""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# ==================== 心跳模拟器 ====================
class HeartbeatSimulator:
    def __init__(self):
        self.heartbeats = []
        self.is_connected = True
        self.last_time = None
        self.running = False
        
    def start(self):
        self.running = True
        self.heartbeats = []
        self.last_time = None
        self.is_connected = True
        
    def stop(self):
        self.running = False
        
    def update(self):
        if not self.running:
            return None
        
        seq = len(self.heartbeats) + 1
        now = time.time()
        heartbeat = {
            "序号": seq,
            "时间": datetime.now().strftime("%H:%M:%S"),
            "延迟": round(random.uniform(10, 50), 1),
            "信号强度": round(random.uniform(70, 100), 1)
        }
        self.heartbeats.append(heartbeat)
        self.last_time = now
        
        # 检查掉线
        return heartbeat
    
    def check_connection(self):
        if self.last_time is None:
            return True
        if time.time() - self.last_time > 3:
            self.is_connected = False
        else:
            self.is_connected = True
        return self.is_connected

# ==================== 页面1：航线规划 ====================
def page_route_planning():
    st.header("🗺️ 航线规划")
    
    # 侧边栏控制
    with st.sidebar:
        st.markdown("### 🎮 控制面板")
        
        # 坐标系选择
        st.markdown("#### 坐标系设置")
        coord_system = st.selectbox(
            "输入坐标系",
            ["GCJ-02 (高德/百度地图)", "WGS-84 (GPS)"],
            key="coord_system"
        )
        
        st.markdown("---")
        
        # A点设置
        st.markdown("#### 起点 A")
        col1, col2 = st.columns(2)
        with col1:
            lat_a = st.number_input("纬度", value=32.2322, format="%.6f", key="lat_a")
        with col2:
            lng_a = st.number_input("经度", value=118.7490, format="%.6f", key="lng_a")
        
        if st.button("📍 设置A点", use_container_width=True):
            st.session_state.waypoints['A'] = {'lat': lat_a, 'lng': lng_a, 'coord': coord_system}
            st.success("✅ A点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # B点设置
        st.markdown("#### 终点 B")
        col3, col4 = st.columns(2)
        with col3:
            lat_b = st.number_input("纬度", value=32.2343, format="%.6f", key="lat_b")
        with col4:
            lng_b = st.number_input("经度", value=118.7510, format="%.6f", key="lng_b")
        
        if st.button("📍 设置B点", use_container_width=True):
            st.session_state.waypoints['B'] = {'lat': lat_b, 'lng': lng_b, 'coord': coord_system}
            st.success("✅ B点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("#### 飞行参数")
        altitude = st.slider("飞行高度 (m)", 20, 200, 50, key="altitude")
        speed = st.slider("飞行速度 (m/s)", 5, 30, 10, key="speed")
        
        # 障碍物设置
        st.markdown("#### 障碍物设置")
        show_obstacles = st.checkbox("显示障碍物", value=True)
    
    # 主显示区域
    # 显示AB点状态
    col_a, col_b, col_info = st.columns(3)
    
    with col_a:
        if st.session_state.waypoints['A']:
            a = st.session_state.waypoints['A']
            st.info(f"📍 **A点**\n\n纬度: {a['lat']:.6f}\n经度: {a['lng']:.6f}\n坐标系: {a['coord']}")
        else:
            st.warning("⚠️ A点未设置")
    
    with col_b:
        if st.session_state.waypoints['B']:
            b = st.session_state.waypoints['B']
            st.info(f"🎯 **B点**\n\n纬度: {b['lat']:.6f}\n经度: {b['lng']:.6f}\n坐标系: {b['coord']}")
        else:
            st.warning("⚠️ B点未设置")
    
    with col_info:
        if st.session_state.waypoints['A'] and st.session_state.waypoints['B']:
            a = st.session_state.waypoints['A']
            b = st.session_state.waypoints['B']
            
            # 坐标转换显示
            if "GCJ-02" in a['coord']:
                wgs_lat, wgs_lng = gcj02_to_wgs84(a['lng'], a['lat'])
                st.metric("A点 WGS-84坐标", f"{wgs_lat:.6f}, {wgs_lng:.6f}")
            else:
                gcj_lat, gcj_lng = wgs84_to_gcj02(a['lng'], a['lat'])
                st.metric("A点 GCJ-02坐标", f"{gcj_lat:.6f}, {gcj_lng:.6f}")
    
    # 3D地图
    st.markdown("### 🗺️ 3D航线地图")
    
    if st.session_state.waypoints['A'] and st.session_state.waypoints['B']:
        a = st.session_state.waypoints['A']
        b = st.session_state.waypoints['B']
        
        # 创建航线点
        lat_points = [a['lat'], b['lat']]
        lng_points = [a['lng'], b['lng']]
        
        # 计算距离
        distance = calculate_distance(a['lat'], a['lng'], b['lat'], b['lng'])
        
        # 添加中间点（模拟航线）
        num_points = 20
        for i in range(num_points + 1):
            t = i / num_points
            lat = a['lat'] + t * (b['lat'] - a['lat'])
            lng = a['lng'] + t * (b['lng'] - a['lng'])
            if 0 < t < 1:
                lat_points.append(lat)
                lng_points.append(lng)
        
        # 创建3D地图
        fig = go.Figure()
        
        # 航线
        fig.add_trace(go.Scatter3d(
            x=lng_points,
            y=lat_points,
            z=[altitude] * len(lng_points),
            mode='lines+markers',
            name='飞行航线',
            line=dict(color='cyan', width=4),
            marker=dict(size=5, color='red')
        ))
        
        # A点标记
        fig.add_trace(go.Scatter3d(
            x=[a['lng']],
            y=[a['lat']],
            z=[altitude],
            mode='markers+text',
            name='起点 A',
            text=['A'],
            textposition='top center',
            marker=dict(size=10, color='green')
        ))
        
        # B点标记
        fig.add_trace(go.Scatter3d(
            x=[b['lng']],
            y=[b['lat']],
            z=[altitude],
            mode='markers+text',
            name='终点 B',
            text=['B'],
            textposition='top center',
            marker=dict(size=10, color='blue')
        ))
        
        # 障碍物
        if show_obstacles:
            obstacles = [
                {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.25, 'lng': a['lng'] + (b['lng'] - a['lng']) * 0.25, 'z': altitude * 0.6},
                {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.5, 'lng': a['lng'] + (b['lng'] - a['lng']) * 0.5 + 0.0005, 'z': altitude * 0.8},
                {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.75, 'lng': a['lng'] + (b['lng'] - a['lng']) * 0.75 - 0.0003, 'z': altitude * 0.7},
            ]
            for i, obs in enumerate(obstacles):
                fig.add_trace(go.Scatter3d(
                    x=[obs['lng']],
                    y=[obs['lat']],
                    z=[obs['z']],
                    mode='markers',
                    name=f'障碍物 {i+1}',
                    marker=dict(size=15, color='orange', symbol='cube')
                ))
        
        fig.update_layout(
            title=f'3D航线规划 (距离: {distance:.0f}米, 高度: {altitude}米)',
            scene=dict(
                xaxis_title='经度',
                yaxis_title='纬度',
                zaxis_title='高度 (米)',
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
            ),
            height=600,
            margin=dict(l=0, r=0, t=50, b=0)
        )
        
        st.plotly_chart(fig, use_container_width=True)
        
        # 飞行信息
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("航线距离", f"{distance:.0f} 米")
        with col2:
            flight_time = distance / speed
            st.metric("预计飞行时间", f"{flight_time:.1f} 秒")
        with col3:
            st.metric("设定高度", f"{altitude} 米")
    
    else:
        st.info("💡 请在左侧设置A点和B点，将显示3D航线图")

# ==================== 页面2：飞行监控 ====================
def page_flight_monitor():
    st.header("📡 飞行监控")
    
    # 初始化心跳模拟器
    if 'heartbeat_sim' not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator()
    
    # 侧边栏
    with st.sidebar:
        st.markdown("### 🎮 监控控制")
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("🚀 开始监控", use_container_width=True):
                st.session_state.heartbeat_sim.start()
                st.session_state.is_flying = True
                st.rerun()
        with col2:
            if st.button("⏹️ 停止监控", use_container_width=True):
                st.session_state.heartbeat_sim.stop()
                st.session_state.is_flying = False
                st.rerun()
        
        st.markdown("---")
        
        if st.button("📡 模拟信号中断", use_container_width=True):
            st.warning("正在模拟信号中断...")
        
        st.markdown("---")
        
        total_packets = len(st.session_state.heartbeat_sim.heartbeats)
        st.metric("累计心跳包", total_packets)
    
    # 状态显示
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.markdown("#### 连接状态")
        if st.session_state.heartbeat_sim.running:
            connected = st.session_state.heartbeat_sim.check_connection()
            if connected:
                st.success("✅ 连接正常")
            else:
                st.error("⚠️ 掉线报警！")
        else:
            st.info("⏸️ 未启动")
    
    with col2:
        st.markdown("#### 最后心跳")
        if st.session_state.heartbeat_sim.heartbeats:
            last = st.session_state.heartbeat_sim.heartbeats[-1]
            st.metric("时间", last['时间'])
        else:
            st.metric("时间", "--")
    
    with col3:
        st.markdown("#### 飞行状态")
        if st.session_state.is_flying:
            st.info("🚁 飞行中")
        else:
            st.info("⏸️ 待机")
    
    # 实时更新
    if st.session_state.heartbeat_sim.running:
        heartbeat = st.session_state.heartbeat_sim.update()
        time.sleep(0.5)
        st.rerun()
    
    # 心跳图表
    st.markdown("### 📈 心跳监测")
    
    data = st.session_state.heartbeat_sim.heartbeats
    if data:
        df = pd.DataFrame(data)
        
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=list(range(1, len(df)+1)),
            y=df['序号'],
            mode='lines+markers',
            name='心跳序号',
            line=dict(color='#00ff00', width=2),
            marker=dict(size=8, color='cyan')
        ))
        fig.update_layout(
            title="心跳包序号变化",
            xaxis_title="时间序列",
            yaxis_title="心跳序号",
            height=400,
            template='plotly_dark'
        )
        st.plotly_chart(fig, use_container_width=True)
        
        # 信号强度
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(
            x=list(range(1, len(df)+1)),
            y=df['信号强度'],
            mode='lines+fill',
            name='信号强度',
            line=dict(color='yellow', width=2),
            fill='tozeroy'
        ))
        fig2.update_layout(
            title="信号强度变化",
            xaxis_title="时间序列",
            yaxis_title="信号强度 (%)",
            height=300,
            template='plotly_dark'
        )
        st.plotly_chart(fig2, use_container_width=True)
        
        # 数据表格
        with st.expander("查看详细数据"):
            st.dataframe(df, use_container_width=True)
    else:
        st.info("💡 点击「开始监控」查看实时心跳数据")

# ==================== 主程序 ====================
def main():
    # 页面选择
    st.sidebar.markdown("# 🚁 无人机系统")
    st.sidebar.markdown("---")
    
    page = st.sidebar.radio(
        "选择功能页面",
        ["🗺️ 航线规划", "📡 飞行监控"],
        format_func=lambda x: x
    )
    
    st.sidebar.markdown("---")
    
    # 显示系统状态
    st.sidebar.markdown("### 📊 系统状态")
    if st.session_state.waypoints['A']:
        st.sidebar.success("✅ A点已设")
    else:
        st.sidebar.warning("⚠️ A点未设")
    
    if st.session_state.waypoints['B']:
        st.sidebar.success("✅ B点已设")
    else:
        st.sidebar.warning("⚠️ B点未设")
    
    # 根据选择显示页面
    if page == "🗺️ 航线规划":
        page_route_planning()
    else:
        page_flight_monitor()

if __name__ == "__main__":
    # 初始化session state
    if 'waypoints' not in st.session_state:
        st.session_state.waypoints = {'A': None, 'B': None}
    if 'is_flying' not in st.session_state:
        st.session_state.is_flying = False
    
    main()
