"""
无人机任务规划与监控系统
包含航线规划（3D地图）和飞行监控（心跳监测）
"""

import streamlit as st
import pandas as pd
import plotly.graph_objects as go
from datetime import datetime
import time
import random
import math

# ==================== 初始化 Session State ====================
def init_session_state():
    """初始化所有session state变量"""
    if 'waypoints' not in st.session_state:
        st.session_state.waypoints = {'A': None, 'B': None}
    if 'is_flying' not in st.session_state:
        st.session_state.is_flying = False
    if 'heartbeat_sim' not in st.session_state:
        st.session_state.heartbeat_sim = None
    if 'coord_system' not in st.session_state:
        st.session_state.coord_system = "GCJ-02 (高德/百度地图)"
    if 'altitude' not in st.session_state:
        st.session_state.altitude = 50
    if 'speed' not in st.session_state:
        st.session_state.speed = 10

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
        self.last_heartbeat = None
        
    def start(self):
        self.running = True
        self.heartbeats = []
        self.last_time = None
        self.is_connected = True
        self.last_heartbeat = None
        
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
            "完整时间": datetime.now(),
            "延迟": round(random.uniform(10, 50), 1),
            "信号强度": round(random.uniform(70, 100), 1)
        }
        self.heartbeats.append(heartbeat)
        self.last_time = now
        self.last_heartbeat = heartbeat
        return heartbeat
    
    def check_connection(self):
        if self.last_time is None:
            return True
        time_since = time.time() - self.last_time
        if time_since > 3:
            self.is_connected = False
        else:
            self.is_connected = True
        return self.is_connected

# ==================== 页面1：航线规划 ====================
def page_route_planning():
    st.header("🗺️ 航线规划")
    
    # 使用两列布局
    left_col, right_col = st.columns([1, 2])
    
    with left_col:
        st.markdown("### 🎮 控制面板")
        
        # 坐标系设置
        st.markdown("#### 坐标系设置")
        coord_system = st.radio(
            "输入坐标系",
            ["GCJ-02 (高德/百度地图)", "WGS-84 (GPS)"],
            index=0,
            key="coord_select"
        )
        st.session_state.coord_system = coord_system
        
        st.markdown("---")
        
        # A点设置
        st.markdown("#### 起点 A")
        col1, col2 = st.columns(2)
        with col1:
            lat_a_input = st.number_input(
                "纬度", 
                value=32.232200, 
                format="%.6f",
                key="lat_a_input",
                step=0.0001
            )
        with col2:
            lng_a_input = st.number_input(
                "经度", 
                value=118.749000, 
                format="%.6f",
                key="lng_a_input",
                step=0.0001
            )
        
        if st.button("📍 设置A点", use_container_width=True, key="set_a"):
            st.session_state.waypoints['A'] = {
                'lat': lat_a_input, 
                'lng': lng_a_input, 
                'coord': coord_system
            }
            st.success("✅ A点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # B点设置
        st.markdown("#### 终点 B")
        col3, col4 = st.columns(2)
        with col3:
            lat_b_input = st.number_input(
                "纬度", 
                value=32.234300, 
                format="%.6f",
                key="lat_b_input",
                step=0.0001
            )
        with col4:
            lng_b_input = st.number_input(
                "经度", 
                value=118.751000, 
                format="%.6f",
                key="lng_b_input",
                step=0.0001
            )
        
        if st.button("📍 设置B点", use_container_width=True, key="set_b"):
            st.session_state.waypoints['B'] = {
                'lat': lat_b_input, 
                'lng': lng_b_input, 
                'coord': coord_system
            }
            st.success("✅ B点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("#### 飞行参数")
        altitude = st.slider("飞行高度 (m)", 20, 200, 50, key="altitude_slider")
        speed = st.slider("飞行速度 (m/s)", 5, 30, 10, key="speed_slider")
        st.session_state.altitude = altitude
        st.session_state.speed = speed
        
        # 障碍物
        show_obstacles = st.checkbox("显示障碍物", value=True, key="show_obs")
    
    with right_col:
        # 显示当前设置的点
        st.markdown("### 📍 当前航点")
        
        col_a, col_b = st.columns(2)
        with col_a:
            if st.session_state.waypoints['A']:
                a = st.session_state.waypoints['A']
                st.info(f"**起点 A**\n\n📍 纬度: {a['lat']:.6f}\n📍 经度: {a['lng']:.6f}\n📐 坐标系: {a['coord'].split()[0]}")
            else:
                st.warning("⚠️ 起点 A 未设置")
        
        with col_b:
            if st.session_state.waypoints['B']:
                b = st.session_state.waypoints['B']
                st.info(f"**终点 B**\n\n📍 纬度: {b['lat']:.6f}\n📍 经度: {b['lng']:.6f}\n📐 坐标系: {b['coord'].split()[0]}")
            else:
                st.warning("⚠️ 终点 B 未设置")
        
        st.markdown("---")
        
        # 3D地图
        st.markdown("### 🗺️ 3D航线地图")
        
        if st.session_state.waypoints['A'] and st.session_state.waypoints['B']:
            a = st.session_state.waypoints['A']
            b = st.session_state.waypoints['B']
            
            # 计算距离
            distance = calculate_distance(a['lat'], a['lng'], b['lat'], b['lng'])
            flight_time = distance / st.session_state.speed
            
            # 显示飞行信息
            info_col1, info_col2, info_col3 = st.columns(3)
            with info_col1:
                st.metric("航线距离", f"{distance:.0f} 米")
            with info_col2:
                st.metric("预计飞行时间", f"{flight_time:.1f} 秒")
            with info_col3:
                st.metric("飞行高度", f"{st.session_state.altitude} 米")
            
            # 创建3D地图
            fig = go.Figure()
            
            # 航线
            fig.add_trace(go.Scatter3d(
                x=[a['lng'], b['lng']],
                y=[a['lat'], b['lat']],
                z=[st.session_state.altitude, st.session_state.altitude],
                mode='lines+markers',
                name='飞行航线',
                line=dict(color='#00ff00', width=5),
                marker=dict(size=8, color='red')
            ))
            
            # A点
            fig.add_trace(go.Scatter3d(
                x=[a['lng']],
                y=[a['lat']],
                z=[st.session_state.altitude],
                mode='markers+text',
                name='起点 A',
                text=['🟢 A'],
                textposition='top center',
                textfont=dict(size=14, color='green'),
                marker=dict(size=12, color='green')
            ))
            
            # B点
            fig.add_trace(go.Scatter3d(
                x=[b['lng']],
                y=[b['lat']],
                z=[st.session_state.altitude],
                mode='markers+text',
                name='终点 B',
                text=['🔴 B'],
                textposition='top center',
                textfont=dict(size=14, color='red'),
                marker=dict(size=12, color='red')
            ))
            
            # 障碍物
            if show_obstacles:
                obstacles = [
                    {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.3, 
                     'lng': a['lng'] + (b['lng'] - a['lng']) * 0.3 + 0.0003, 
                     'z': st.session_state.altitude * 0.6},
                    {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.5, 
                     'lng': a['lng'] + (b['lng'] - a['lng']) * 0.5, 
                     'z': st.session_state.altitude * 0.8},
                    {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.7, 
                     'lng': a['lng'] + (b['lng'] - a['lng']) * 0.7 - 0.0004, 
                     'z': st.session_state.altitude * 0.7},
                ]
                for i, obs in enumerate(obstacles):
                    fig.add_trace(go.Scatter3d(
                        x=[obs['lng']],
                        y=[obs['lat']],
                        z=[obs['z']],
                        mode='markers+text',
                        name=f'障碍物 {i+1}',
                        text=[f'🧱 {i+1}'],
                        textposition='top center',
                        marker=dict(size=10, color='orange', symbol='cube')
                    ))
            
            fig.update_layout(
                title='<b>3D 航线规划图</b>',
                scene=dict(
                    xaxis_title='经度',
                    yaxis_title='纬度',
                    zaxis_title='高度 (米)',
                    camera=dict(eye=dict(x=1.5, y=1.5, z=1.5)),
                    aspectmode='manual',
                    aspectratio=dict(x=1, y=1, z=0.5)
                ),
                height=550,
                margin=dict(l=0, r=0, t=50, b=0),
                showlegend=True
            )
            
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("💡 请在左侧设置A点和B点，将显示3D航线图")
            
            # 显示示例提示
            st.markdown("""
            ### 📝 使用说明
            1. 在左侧面板选择坐标系
            2. 输入A点（起点）的经纬度
            3. 输入B点（终点）的经纬度
            4. 点击「设置A点」和「设置B点」按钮
            5. 调整飞行高度和速度
            6. 查看3D航线图
            """)

# ==================== 页面2：飞行监控 ====================
def page_flight_monitor():
    st.header("📡 飞行监控")
    
    # 初始化心跳模拟器
    if st.session_state.heartbeat_sim is None:
        st.session_state.heartbeat_sim = HeartbeatSimulator()
    
    # 侧边栏控制
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
        
        # 模拟掉线
        if st.button("📡 模拟掉线", use_container_width=True):
            st.session_state.heartbeat_sim.last_time = time.time() - 5
            st.warning("⚠️ 模拟信号中断...")
            time.sleep(0.5)
            st.rerun()
        
        st.markdown("---")
        
        # 统计
        total = len(st.session_state.heartbeat_sim.heartbeats)
        st.metric("累计心跳包", total)
    
    # 状态显示
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.markdown("#### 🔗 连接状态")
        if st.session_state.heartbeat_sim.running:
            connected = st.session_state.heartbeat_sim.check_connection()
            if connected:
                st.success("✅ 连接正常")
            else:
                st.error("⚠️ 掉线报警！")
        else:
            st.info("⏸️ 未启动")
    
    with col2:
        st.markdown("#### ⏰ 最后心跳")
        if st.session_state.heartbeat_sim.last_heartbeat:
            last = st.session_state.heartbeat_sim.last_heartbeat
            st.metric("时间", last['时间'])
            st.metric("信号强度", f"{last['信号强度']}%")
        else:
            st.metric("时间", "--")
    
    with col3:
        st.markdown("#### 🚁 飞行状态")
        if st.session_state.is_flying:
            if st.session_state.heartbeat_sim.check_connection():
                st.info("✈️ 飞行中")
            else:
                st.error("⚠️ 紧急降落")
        else:
            st.info("⏸️ 待机")
    
    # 实时更新
    if st.session_state.heartbeat_sim.running:
        heartbeat = st.session_state.heartbeat_sim.update()
        time.sleep(0.5)
        st.rerun()
    
    # 图表
    st.markdown("### 📈 心跳监测数据")
    
    data = st.session_state.heartbeat_sim.heartbeats
    if data:
        df = pd.DataFrame(data)
        
        # 心跳序号图
        fig1 = go.Figure()
        fig1.add_trace(go.Scatter(
            x=df['序号'],
            y=df['序号'],
            mode='lines+markers',
            name='心跳序号',
            line=dict(color='#00ff00', width=2),
            marker=dict(size=8, color='cyan')
        ))
        fig1.update_layout(
            title="心跳包序号变化",
            xaxis_title="时间序列",
            yaxis_title="心跳序号",
            height=350,
            template='plotly_dark'
        )
        st.plotly_chart(fig1, use_container_width=True)
        
        # 信号强度图
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(
            x=df['序号'],
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
        with st.expander("📋 查看详细数据", expanded=False):
            st.dataframe(df[['序号', '时间', '信号强度', '延迟']], use_container_width=True)
    else:
        st.info("💡 点击「开始监控」查看实时心跳数据")

# ==================== 主程序 ====================
def main():
    # 初始化
    init_session_state()
    
    # 页面配置
    st.set_page_config(
        page_title="无人机任务规划系统",
        page_icon="🚁",
        layout="wide",
        initial_sidebar_state="expanded"
    )
    
    # 标题
    st.title("🚁 无人机智能化任务规划系统")
    st.markdown("---")
    
    # 页面选择
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🗺️ 航线规划", use_container_width=True):
            st.session_state.page = "route"
    with col2:
        if st.button("📡 飞行监控", use_container_width=True):
            st.session_state.page = "monitor"
    
    if 'page' not in st.session_state:
        st.session_state.page = "route"
    
    st.markdown("---")
    
    # 显示当前页面
    if st.session_state.page == "route":
        page_route_planning()
    else:
        page_flight_monitor()
    
    # 页脚
    st.markdown("---")
    st.markdown(
        """
        <div style="text-align: center; color: #666; padding: 1rem;">
        🚁 无人机任务规划系统 | 支持坐标转换 | 3D航线规划 | 实时心跳监测
        </div>
        """,
        unsafe_allow_html=True
    )

if __name__ == "__main__":
    main()
