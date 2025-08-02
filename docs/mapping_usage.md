# TransBot 建图功能使用说明

## 启动方式

### 1. 纯建图模式 (推荐)
```bash
roslaunch transbot_manager start_all.launch with_navigation:=false with_mapping:=true map_type:=cartographer open_rviz:=true
```

### 2. 导航+建图模式 (同时运行)
```bash
roslaunch transbot_manager start_all.launch with_navigation:=true with_mapping:=true map_type:=cartographer map_name:=new_map open_rviz:=true
```

### 3. 纯导航模式 (默认)
```bash
roslaunch transbot_manager start_all.launch with_navigation:=true with_mapping:=false map_name:=house
```

## 参数说明

- `with_navigation`: 是否启动导航系统
- `with_mapping`: 是否启动建图系统  
- `map_type`: 建图算法 (cartographer/gmapping/hector/karto)
- `map_name`: 地图名称
- `open_rviz`: 是否打开可视化

## 保存地图

### 方法1: 使用ROS服务
```bash
# 保存为默认名称
rosservice call /mapping_manager/save_map

# 先设置地图名，再保存
rosparam set /mapping_manager/map_name "office_map"
rosservice call /mapping_manager/save_map
```

### 方法2: 使用工具脚本
```bash
# 保存为默认名称
rosrun transbot_manager save_map_tool.py

# 保存为指定名称
rosrun transbot_manager save_map_tool.py office_map
```

## 建图状态监控

```bash
# 查看建图状态
rostopic echo /mapping_manager/mapping_status
```

状态说明:
- `ready`: 系统就绪
- `mapping`: 正在建图
- `saving`: 正在保存地图
- `saved`: 地图已保存
- `error`: 出现错误

## 冲突处理

导航和建图可以同时启动，系统会自动处理冲突:
1. 传感器数据共享 (激光雷达、相机)
2. 建图时不会启动定位和路径规划
3. 导航时可以查看实时建图结果

## 文件位置

保存的地图文件位于: `~/transbot_ws/src/transbot_nav/maps/`

每个地图包含:
- `map_name.yaml` - 地图配置文件
- `map_name.pgm` - 地图图像文件  
- `map_name.pbstream` - Cartographer原始数据 (仅cartographer)

## 常见问题

1. **建图服务不可用**: 确保 `with_mapping:=true`
2. **保存失败**: 检查地图目录权限和磁盘空间
3. **rviz无法显示**: 检查话题名称和数据类型
