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

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="无人机任务规划系统",
    page_icon="🚁",
    layout="wide"
)

# ==================== 初始化 Session State ====================
if 'waypoint_a' not in st.session_state:
    st.session_state.waypoint_a = None
if 'waypoint_b' not in st.session_state:
    st.session_state.waypoint_b = None
if 'coord_system' not in st.session_state:
    st.session_state.coord_system = "GCJ-02"
if 'altitude' not in st.session_state:
    st.session_state.altitude = 50
if 'speed' not in st.session_state:
    st.session_state.speed = 10
if 'is_flying' not in st.session_state:
    st.session_state.is_flying = False
if 'heartbeats' not in st.session_state:
    st.session_state.heartbeats = []
if 'monitoring' not in st.session_state:
    st.session_state.monitoring = False
if 'last_heartbeat_time' not in st.session_state:
    st.session_state.last_heartbeat_time = None
if 'page' not in st.session_state:
    st.session_state.page = "航线规划"

# ==================== 坐标系转换 ====================
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
    return lng * 2 - dlng, lat * 2 - dlat

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

# ==================== 心跳模拟 ====================
def update_heartbeat():
    """更新心跳数据"""
    if not st.session_state.monitoring:
        return None
    
    seq = len(st.session_state.heartbeats) + 1
    now = time.time()
    heartbeat = {
        "序号": seq,
        "时间": datetime.now().strftime("%H:%M:%S"),
        "延迟": round(random.uniform(10, 50), 1),
        "信号强度": round(random.uniform(70, 100), 1)
    }
    st.session_state.heartbeats.append(heartbeat)
    st.session_state.last_heartbeat_time = now
    return heartbeat

def check_connection():
    """检查连接状态"""
    if st.session_state.last_heartbeat_time is None:
        return True
    if time.time() - st.session_state.last_heartbeat_time > 3:
        return False
    return True

# ==================== 页面1：航线规划 ====================
def page_route_planning():
    st.header("🗺️ 航线规划")
    
    # 使用简单布局
    st.markdown("### 📍 航点设置")
    
    # 坐标系选择
    st.markdown("#### 坐标系设置")
    coord_system = st.radio(
        "选择坐标系",
        ["GCJ-02 (高德/百度地图)", "WGS-84 (GPS)"],
        horizontal=True
    )
    st.session_state.coord_system = coord_system.split()[0]
    
    st.markdown("---")
    
    # A点设置
    st.markdown("#### 起点 A")
    col1, col2 = st.columns(2)
    with col1:
        lat_a = st.number_input("纬度", value=32.2322, format="%.6f", key="lat_a")
    with col2:
        lng_a = st.number_input("经度", value=118.7490, format="%.6f", key="lng_a")
    
    if st.button("✅ 设置A点", use_container_width=True):
        st.session_state.waypoint_a = {"lat": lat_a, "lng": lng_a}
        st.success(f"✅ A点已设置: ({lat_a:.6f}, {lng_a:.6f})")
        st.rerun()
    
    st.markdown("---")
    
    # B点设置
    st.markdown("#### 终点 B")
    col3, col4 = st.columns(2)
    with col3:
        lat_b = st.number_input("纬度", value=32.2343, format="%.6f", key="lat_b")
    with col4:
        lng_b = st.number_input("经度", value=118.7510, format="%.6f", key="lng_b")
    
    if st.button("🎯 设置B点", use_container_width=True):
        st.session_state.waypoint_b = {"lat": lat_b, "lng": lng_b}
        st.success(f"✅ B点已设置: ({lat_b:.6f}, {lng_b:.6f})")
        st.rerun()
    
    st.markdown("---")
    
    # 飞行参数
    st.markdown("#### 飞行参数")
    col5, col6 = st.columns(2)
    with col5:
        altitude = st.slider("飞行高度 (米)", 20, 200, st.session_state.altitude)
        st.session_state.altitude = altitude
    with col6:
        speed = st.slider("飞行速度 (米/秒)", 5, 30, st.session_state.speed)
        st.session_state.speed = speed
    
    st.markdown("---")
    
    # 显示当前航点状态
    st.markdown("### 📊 当前航点状态")
    status_col1, status_col2 = st.columns(2)
    with status_col1:
        if st.session_state.waypoint_a:
            a = st.session_state.waypoint_a
            st.success(f"**起点 A**\n\n纬度: {a['lat']:.6f}\n经度: {a['lng']:.6f}")
        else:
            st.warning("⚠️ 起点 A 未设置")
    
    with status_col2:
        if st.session_state.waypoint_b:
            b = st.session_state.waypoint_b
            st.success(f"**终点 B**\n\n纬度: {b['lat']:.6f}\n经度: {b['lng']:.6f}")
        else:
            st.warning("⚠️ 终点 B 未设置")
    
    st.markdown("---")
    
    # 3D地图
    st.markdown("### 🗺️ 3D航线地图")
    
    if st.session_state.waypoint_a and st.session_state.waypoint_b:
        a = st.session_state.waypoint_a
        b = st.session_state.waypoint_b
        
        # 计算距离
        distance = calculate_distance(a['lat'], a['lng'], b['lat'], b['lng'])
        flight_time = distance / st.session_state.speed
        
        # 显示信息
        info_col1, info_col2, info_col3, info_col4 = st.columns(4)
        with info_col1:
            st.metric("航线距离", f"{distance:.0f} 米")
        with info_col2:
            st.metric("预计时间", f"{flight_time:.1f} 秒")
        with info_col3:
            st.metric("飞行高度", f"{st.session_state.altitude} 米")
        with info_col4:
            st.metric("飞行速度", f"{st.session_state.speed} 米/秒")
        
        # 创建3D地图
        fig = go.Figure()
        
        # 航线
        fig.add_trace(go.Scatter3d(
            x=[a['lng'], b['lng']],
            y=[a['lat'], b['lat']],
            z=[st.session_state.altitude, st.session_state.altitude],
            mode='lines+markers',
            name='飞行航线',
            line=dict(color='#00ff00', width=6),
            marker=dict(size=10, color='red')
        ))
        
        # A点
        fig.add_trace(go.Scatter3d(
            x=[a['lng']],
            y=[a['lat']],
            z=[st.session_state.altitude],
            mode='markers+text',
            name='起点 A',
            text=['🟢 起点 A'],
            textposition='top center',
            marker=dict(size=15, color='green')
        ))
        
        # B点
        fig.add_trace(go.Scatter3d(
            x=[b['lng']],
            y=[b['lat']],
            z=[st.session_state.altitude],
            mode='markers+text',
            name='终点 B',
            text=['🔴 终点 B'],
            textposition='top center',
            marker=dict(size=15, color='red')
        ))
        
        # 障碍物
        obstacles = [
            {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.25, 
             'lng': a['lng'] + (b['lng'] - a['lng']) * 0.25 + 0.0005, 
             'z': st.session_state.altitude * 0.5},
            {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.5, 
             'lng': a['lng'] + (b['lng'] - a['lng']) * 0.5, 
             'z': st.session_state.altitude * 0.7},
            {'lat': a['lat'] + (b['lat'] - a['lat']) * 0.75, 
             'lng': a['lng'] + (b['lng'] - a['lng']) * 0.75 - 0.0003, 
             'z': st.session_state.altitude * 0.6},
        ]
        
        for i, obs in enumerate(obstacles):
            fig.add_trace(go.Scatter3d(
                x=[obs['lng']],
                y=[obs['lat']],
                z=[obs['z']],
                mode='markers',
                name=f'障碍物 {i+1}',
                marker=dict(size=12, color='orange', symbol='cube')
            ))
        
        fig.update_layout(
            title='<b>3D 航线规划图</b>',
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
        
        # 坐标转换显示
        st.markdown("### 🔄 坐标转换")
        if "GCJ-02" in st.session_state.coord_system:
            wgs_lat, wgs_lng = gcj02_to_wgs84(a['lng'], a['lat'])
            st.info(f"📍 A点 WGS-84坐标: ({wgs_lat:.6f}, {wgs_lng:.6f})")
        else:
            gcj_lat, gcj_lng = wgs84_to_gcj02(a['lng'], a['lat'])
            st.info(f"📍 A点 GCJ-02坐标: ({gcj_lat:.6f}, {gcj_lng:.6f})")
    
    else:
        st.info("💡 请先设置A点和B点，将显示3D航线图")
        st.markdown("""
        ### 📝 使用说明
        1. 选择坐标系（GCJ-02 或 WGS-84）
        2. 输入A点（起点）的经纬度
        3. 点击「设置A点」按钮
        4. 输入B点（终点）的经纬度
        5. 点击「设置B点」按钮
        6. 调整飞行高度和速度
        7. 查看3D航线图
        """)

# ==================== 页面2：飞行监控 ====================
def page_flight_monitor():
    st.header("📡 飞行监控")
    
    # 控制按钮
    col1, col2, col3 = st.columns(3)
    with col1:
        if st.button("🚀 开始监控", use_container_width=True):
            st.session_state.monitoring = True
            st.session_state.heartbeats = []
            st.session_state.last_heartbeat_time = None
            st.rerun()
    with col2:
        if st.button("⏹️ 停止监控", use_container_width=True):
            st.session_state.monitoring = False
            st.rerun()
    with col3:
        if st.button("📡 模拟掉线", use_container_width=True):
            st.session_state.last_heartbeat_time = time.time() - 5
            st.warning("⚠️ 模拟信号中断！")
    
    st.markdown("---")
    
    # 状态显示
    st.markdown("### 📊 系统状态")
    status_col1, status_col2, status_col3 = st.columns(3)
    
    with status_col1:
        if st.session_state.monitoring:
            connected = check_connection()
            if connected:
                st.success("✅ 连接正常")
            else:
                st.error("⚠️ 掉线报警！")
        else:
            st.info("⏸️ 监控未启动")
    
    with status_col2:
        if st.session_state.heartbeats:
            last = st.session_state.heartbeats[-1]
            st.metric("最后心跳时间", last['时间'])
        else:
            st.metric("最后心跳时间", "--")
    
    with status_col3:
        st.metric("累计心跳包", len(st.session_state.heartbeats))
    
    # 实时更新
    if st.session_state.monitoring:
        heartbeat = update_heartbeat()
        time.sleep(0.5)
        st.rerun()
    
    st.markdown("---")
    
    # 图表显示
    st.markdown("### 📈 心跳监测图表")
    
    if st.session_state.heartbeats:
        df = pd.DataFrame(st.session_state.heartbeats)
        
        # 心跳序号图
        fig1 = go.Figure()
        fig1.add_trace(go.Scatter(
            x=df['序号'],
            y=df['序号'],
            mode='lines+markers',
            name='心跳序号',
            line=dict(color='#00ff00', width=3),
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
        with st.expander("📋 查看详细数据"):
            st.dataframe(df, use_container_width=True)
    else:
        st.info("💡 点击「开始监控」查看实时心跳数据")

# ==================== 主程序 ====================
def main():
    # 标题
    st.title("🚁 无人机智能化任务规划系统")
    st.markdown("---")
    
    # 页面切换按钮
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🗺️ 航线规划", use_container_width=True):
            st.session_state.page = "航线规划"
            st.rerun()
    with col2:
        if st.button("📡 飞行监控", use_container_width=True):
            st.session_state.page = "飞行监控"
            st.rerun()
    
    st.markdown("---")
    
    # 显示当前页面
    if st.session_state.page == "航线规划":
        page_route_planning()
    else:
        page_flight_monitor()
    
    # 页脚
    st.markdown("---")
    st.caption("🚁 无人机任务规划系统 | 支持坐标转换 | 3D航线规划 | 实时心跳监测")

if __name__ == "__main__":
    main()
