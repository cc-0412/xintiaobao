"""
无人机智能化任务规划系统
包含：3D地图、2D地图障碍物圈选、坐标转换、心跳监控
"""

import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
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
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []  # 存储障碍物列表
if 'coord_system' not in st.session_state:
    st.session_state.coord_system = "GCJ-02"
if 'altitude' not in st.session_state:
    st.session_state.altitude = 50
if 'speed' not in st.session_state:
    st.session_state.speed = 10
if 'heartbeats' not in st.session_state:
    st.session_state.heartbeats = []
if 'monitoring' not in st.session_state:
    st.session_state.monitoring = False
if 'last_heartbeat_time' not in st.session_state:
    st.session_state.last_heartbeat_time = None
if 'page' not in st.session_state:
    st.session_state.page = "航线规划"
if 'map_zoom' not in st.session_state:
    st.session_state.map_zoom = 15
if 'selected_point' not in st.session_state:
    st.session_state.selected_point = None

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
        "延迟_ms": round(random.uniform(10, 50), 1),
        "信号强度": round(random.uniform(70, 100), 1),
        "电压": round(random.uniform(11.1, 12.6), 1),
        "温度": round(random.uniform(25, 45), 1)
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
    
    # 使用选项卡
    tab1, tab2, tab3 = st.tabs(["📍 航点设置", "🗺️ 2D地图（障碍物圈选）", "🌍 3D航线图"])
    
    # ========== 选项卡1：航点设置 ==========
    with tab1:
        st.markdown("### 📍 航点设置")
        
        # 坐标系选择
        st.markdown("#### 🔄 坐标系设置")
        coord_system = st.radio(
            "输入坐标系",
            ["GCJ-02 (高德/百度地图)", "WGS-84 (GPS)"],
            horizontal=True,
            key="coord_radio"
        )
        st.session_state.coord_system = coord_system.split()[0]
        
        st.markdown("---")
        
        # A点和B点设置 - 两列布局
        col1, col2 = st.columns(2)
        
        with col1:
            st.markdown("#### 🟢 起点 A")
            lat_a = st.number_input("纬度", value=32.2322, format="%.6f", key="lat_a")
            lng_a = st.number_input("经度", value=118.7490, format="%.6f", key="lng_a")
            
            if st.button("✅ 设置A点", use_container_width=True, key="set_a"):
                st.session_state.waypoint_a = {"lat": lat_a, "lng": lng_a}
                st.success(f"✅ A点已设置: ({lat_a:.6f}, {lng_a:.6f})")
                st.rerun()
        
        with col2:
            st.markdown("#### 🔴 终点 B")
            lat_b = st.number_input("纬度", value=32.2343, format="%.6f", key="lat_b")
            lng_b = st.number_input("经度", value=118.7510, format="%.6f", key="lng_b")
            
            if st.button("✅ 设置B点", use_container_width=True, key="set_b"):
                st.session_state.waypoint_b = {"lat": lat_b, "lng": lng_b}
                st.success(f"✅ B点已设置: ({lat_b:.6f}, {lng_b:.6f})")
                st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("#### ⚙️ 飞行参数")
        col3, col4, col5 = st.columns(3)
        with col3:
            altitude = st.slider("飞行高度 (米)", 20, 200, st.session_state.altitude)
            st.session_state.altitude = altitude
        with col4:
            speed = st.slider("飞行速度 (米/秒)", 5, 30, st.session_state.speed)
            st.session_state.speed = speed
        with col5:
            if st.session_state.waypoint_a and st.session_state.waypoint_b:
                dist = calculate_distance(
                    st.session_state.waypoint_a['lat'], st.session_state.waypoint_a['lng'],
                    st.session_state.waypoint_b['lat'], st.session_state.waypoint_b['lng']
                )
                st.metric("航线距离", f"{dist:.0f} 米")
        
        st.markdown("---")
        
        # 当前航点状态
        st.markdown("#### 📊 系统状态")
        status_col1, status_col2 = st.columns(2)
        with status_col1:
            if st.session_state.waypoint_a:
                a = st.session_state.waypoint_a
                st.success(f"🟢 **A点已设**\n\n纬度: {a['lat']:.6f}\n经度: {a['lng']:.6f}")
            else:
                st.warning("⚠️ A点未设")
        
        with status_col2:
            if st.session_state.waypoint_b:
                b = st.session_state.waypoint_b
                st.success(f"🔴 **B点已设**\n\n纬度: {b['lat']:.6f}\n经度: {b['lng']:.6f}")
            else:
                st.warning("⚠️ B点未设")
    
    # ========== 选项卡2：2D地图（障碍物圈选） ==========
    with tab2:
        st.markdown("### 🗺️ 2D地图 - 障碍物圈选")
        st.info("💡 提示：在下方地图上点击可以添加障碍物，点击已添加的障碍物可以删除")
        
        # 显示当前障碍物
        if st.session_state.obstacles:
            st.markdown(f"**当前障碍物数量: {len(st.session_state.obstacles)}**")
            for i, obs in enumerate(st.session_state.obstacles):
                st.caption(f"障碍物 {i+1}: ({obs['lat']:.6f}, {obs['lng']:.6f})")
        
        # 清空障碍物按钮
        if st.button("🗑️ 清空所有障碍物", use_container_width=True):
            st.session_state.obstacles = []
            st.rerun()
        
        st.markdown("---")
        
        # 创建2D地图用于障碍物圈选
        if st.session_state.waypoint_a and st.session_state.waypoint_b:
            a = st.session_state.waypoint_a
            b = st.session_state.waypoint_b
            
            # 计算地图中心
            center_lat = (a['lat'] + b['lat']) / 2
            center_lng = (a['lng'] + b['lng']) / 2
            
            # 创建2D地图
            fig2d = go.Figure()
            
            # 添加航线
            fig2d.add_trace(go.Scattermapbox(
                lat=[a['lat'], b['lat']],
                lon=[a['lng'], b['lng']],
                mode='lines+markers',
                name='航线',
                line=dict(width=3, color='cyan'),
                marker=dict(size=12, color=['green', 'red']),
                text=['起点 A', '终点 B']
            ))
            
            # 添加障碍物
            for i, obs in enumerate(st.session_state.obstacles):
                fig2d.add_trace(go.Scattermapbox(
                    lat=[obs['lat']],
                    lon=[obs['lng']],
                    mode='markers',
                    name=f'障碍物 {i+1}',
                    marker=dict(size=15, color='orange', symbol='circle'),
                    text=[f'障碍物 {i+1}']
                ))
            
            fig2d.update_layout(
                mapbox=dict(
                    style='open-street-map',
                    center=dict(lat=center_lat, lon=center_lng),
                    zoom=st.session_state.map_zoom,
                ),
                height=600,
                margin=dict(l=0, r=0, t=0, b=0)
            )
            
            st.plotly_chart(fig2d, use_container_width=True)
            
            # 缩放控制
            zoom = st.slider("地图缩放级别", 10, 20, st.session_state.map_zoom)
            st.session_state.map_zoom = zoom
            
            # 手动添加障碍物
            st.markdown("#### ➕ 手动添加障碍物")
            col_lat, col_lng, col_btn = st.columns([2, 2, 1])
            with col_lat:
                obs_lat = st.number_input("障碍物纬度", value=center_lat, format="%.6f", key="obs_lat")
            with col_lng:
                obs_lng = st.number_input("障碍物经度", value=center_lng, format="%.6f", key="obs_lng")
            with col_btn:
                if st.button("➕ 添加障碍物", use_container_width=True):
                    st.session_state.obstacles.append({"lat": obs_lat, "lng": obs_lng})
                    st.success("障碍物已添加")
                    st.rerun()
            
        else:
            st.warning("⚠️ 请先在「航点设置」中设置A点和B点")
    
    # ========== 选项卡3：3D航线图 ==========
    with tab3:
        st.markdown("### 🌍 3D航线图")
        
        if st.session_state.waypoint_a and st.session_state.waypoint_b:
            a = st.session_state.waypoint_a
            b = st.session_state.waypoint_b
            
            # 计算距离和飞行时间
            distance = calculate_distance(a['lat'], a['lng'], b['lat'], b['lng'])
            flight_time = distance / st.session_state.speed
            
            # 显示飞行信息
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
            fig3d = go.Figure()
            
            # 航线
            fig3d.add_trace(go.Scatter3d(
                x=[a['lng'], b['lng']],
                y=[a['lat'], b['lat']],
                z=[st.session_state.altitude, st.session_state.altitude],
                mode='lines+markers',
                name='飞行航线',
                line=dict(color='#00ff00', width=6),
                marker=dict(size=10, color=['green', 'red'])
            ))
            
            # A点
            fig3d.add_trace(go.Scatter3d(
                x=[a['lng']],
                y=[a['lat']],
                z=[st.session_state.altitude],
                mode='markers+text',
                name='起点 A',
                text=['🟢 A'],
                textposition='top center',
                marker=dict(size=15, color='green')
            ))
            
            # B点
            fig3d.add_trace(go.Scatter3d(
                x=[b['lng']],
                y=[b['lat']],
                z=[st.session_state.altitude],
                mode='markers+text',
                name='终点 B',
                text=['🔴 B'],
                textposition='top center',
                marker=dict(size=15, color='red')
            ))
            
            # 障碍物（3D显示）
            for i, obs in enumerate(st.session_state.obstacles):
                # 计算障碍物高度（根据距离A点的比例）
                total_dist = distance
                a_to_obs = calculate_distance(a['lat'], a['lng'], obs['lat'], obs['lng'])
                ratio = min(1, a_to_obs / total_dist) if total_dist > 0 else 0.5
                obs_height = st.session_state.altitude * (0.5 + ratio * 0.3)
                
                fig3d.add_trace(go.Scatter3d(
                    x=[obs['lng']],
                    y=[obs['lat']],
                    z=[obs_height],
                    mode='markers+text',
                    name=f'障碍物 {i+1}',
                    text=[f'🧱 {i+1}'],
                    textposition='top center',
                    marker=dict(size=12, color='orange', symbol='cube')
                ))
            
            fig3d.update_layout(
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
            
            st.plotly_chart(fig3d, use_container_width=True)
            
            # 坐标转换显示
            st.markdown("#### 🔄 坐标转换")
            if "GCJ-02" in st.session_state.coord_system:
                wgs_lat, wgs_lng = gcj02_to_wgs84(a['lng'], a['lat'])
                st.info(f"📍 A点 WGS-84坐标: ({wgs_lat:.6f}, {wgs_lng:.6f})")
                wgs_lat_b, wgs_lng_b = gcj02_to_wgs84(b['lng'], b['lat'])
                st.info(f"📍 B点 WGS-84坐标: ({wgs_lat_b:.6f}, {wgs_lng_b:.6f})")
            else:
                gcj_lat, gcj_lng = wgs84_to_gcj02(a['lng'], a['lat'])
                st.info(f"📍 A点 GCJ-02坐标: ({gcj_lat:.6f}, {gcj_lng:.6f})")
                gcj_lat_b, gcj_lng_b = wgs84_to_gcj02(b['lng'], b['lat'])
                st.info(f"📍 B点 GCJ-02坐标: ({gcj_lat_b:.6f}, {gcj_lng_b:.6f})")
        
        else:
            st.info("💡 请先在「航点设置」中设置A点和B点")

# ==================== 页面2：飞行监控 ====================
def page_flight_monitor():
    st.header("📡 飞行监控")
    
    # 控制面板
    st.markdown("### 🎮 监控控制")
    col1, col2, col3, col4 = st.columns(4)
    
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
    
    with col4:
        if st.button("🗑️ 清空数据", use_container_width=True):
            st.session_state.heartbeats = []
            st.rerun()
    
    st.markdown("---")
    
    # 实时更新
    if st.session_state.monitoring:
        heartbeat = update_heartbeat()
        time.sleep(0.5)
        st.rerun()
    
    # 状态显示
    st.markdown("### 📊 系统状态")
    
    status_col1, status_col2, status_col3, status_col4 = st.columns(4)
    
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
    
    with status_col4:
        if st.session_state.heartbeats:
            last = st.session_state.heartbeats[-1]
            st.metric("当前信号强度", f"{last['信号强度']}%")
        else:
            st.metric("当前信号强度", "--")
    
    st.markdown("---")
    
    # 图表显示
    if st.session_state.heartbeats:
        df = pd.DataFrame(st.session_state.heartbeats)
        
        # 心跳序号图
        st.markdown("### 📈 心跳包序号变化")
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
            xaxis_title="时间序列",
            yaxis_title="心跳序号",
            height=350,
            template='plotly_dark'
        )
        st.plotly_chart(fig1, use_container_width=True)
        
        # 信号强度图
        st.markdown("### 📶 信号强度变化")
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
            xaxis_title="时间序列",
            yaxis_title="信号强度 (%)",
            height=300,
            template='plotly_dark'
        )
        st.plotly_chart(fig2, use_container_width=True)
        
        # 延迟变化图
        st.markdown("### ⏱️ 延迟变化")
        fig3 = go.Figure()
        fig3.add_trace(go.Scatter(
            x=df['序号'],
            y=df['延迟_ms'],
            mode='lines+markers',
            name='延迟',
            line=dict(color='orange', width=2),
            marker=dict(size=6)
        ))
        fig3.update_layout(
            xaxis_title="时间序列",
            yaxis_title="延迟 (ms)",
            height=300,
            template='plotly_dark'
        )
        st.plotly_chart(fig3, use_container_width=True)
        
        # 数据表格
        with st.expander("📋 查看详细数据", expanded=False):
            st.dataframe(df, use_container_width=True)
            
            # 导出功能
            csv = df.to_csv(index=False)
            st.download_button(
                label="📥 导出心跳数据",
                data=csv,
                file_name=f"heartbeat_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                mime="text/csv"
            )
    else:
        st.info("💡 点击「开始监控」查看实时心跳数据")
        
        # 显示示例
        st.markdown("### 📋 心跳数据说明")
        st.markdown("""
        - **心跳包**：每秒发送一次，包含序号、时间、信号强度等
        - **掉线检测**：连续3秒未收到心跳包触发报警
        - **信号强度**：模拟70-100%的信号质量
        - **延迟**：模拟10-50ms的网络延迟
        """)

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
    st.caption("🚁 无人机任务规划系统 | 支持GCJ-02/WGS-84坐标转换 | 3D/2D地图 | 实时心跳监测")

if __name__ == "__main__":
    main()
