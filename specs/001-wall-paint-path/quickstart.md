# 快速開始：牆面噴漆路徑規劃

**功能分支**：`001-wall-paint-path`

## 前置需求

```bash
# ROS2 Jazzy 已安裝
source /opt/ros/jazzy/setup.bash

# 安裝 UR10 驅動與 MoveIt2
sudo apt install ros-jazzy-ur ros-jazzy-moveit ros-jazzy-ros2-controllers

# 安裝 Gazebo Harmonic 整合（模擬用）
sudo apt install ros-jazzy-ros-gz
```

## 建置工作區

```bash
cd ~/Desktop/moveit2_ws
colcon build --symlink-install
source install/setup.bash
```

## 啟動模擬環境

```bash
# 步驟 1：啟動 UR10 MoveIt2 模擬
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true

# 步驟 2：啟動噴漆路徑規劃節點（待實作）
ros2 launch wall_paint_planner wall_paint_demo.launch.py
```

## 執行噴漆任務

```bash
# 呼叫路徑規劃服務（範例：3m x 2.5m 牆面）
ros2 service call /paint_path/plan wall_paint_interfaces/srv/PlanPaintPath \
  "{wall_width: 3.0, wall_height: 2.5, strip_overlap: 0.15}"

# 或透過 Action 執行完整任務
ros2 action send_goal /execute_paint_task wall_paint_interfaces/action/ExecutePaintTask \
  "{wall_width: 3.0, wall_height: 2.5, strip_overlap: 0.15}"
```

## 開發流程

1. **牆面路徑規劃器**：實作 zig-zag 路徑點產生演算法
2. **路徑驗證**：透過 MoveIt2 IK 驗證所有路徑點可達性
3. **Pilz 執行器**：使用 Pilz LIN 指令串接路徑點
4. **底盤控制**：實作簡單的左右平移控制
5. **任務協調**：整合手臂動作與底盤移動的順序執行

## 專案結構（預期）

```
src/
├── wall_paint_planner/        # 路徑規劃套件
│   ├── src/
│   │   ├── path_generator.cpp    # zig-zag 路徑點產生
│   │   ├── path_validator.cpp    # IK 可達性驗證
│   │   └── planner_node.cpp      # ROS2 節點
│   ├── launch/
│   │   └── wall_paint_demo.launch.py
│   ├── config/
│   │   └── planner_params.yaml
│   ├── CMakeLists.txt
│   └── package.xml
│
├── wall_paint_coordinator/    # 任務協調套件
│   ├── src/
│   │   ├── task_coordinator.cpp  # 順序執行邏輯
│   │   └── base_controller.cpp   # 底盤平移控制
│   ├── CMakeLists.txt
│   └── package.xml
│
├── wall_paint_interfaces/     # 自訂訊息/服務/動作
│   ├── srv/
│   │   ├── PlanPaintPath.srv
│   │   └── ValidatePath.srv
│   ├── action/
│   │   └── ExecutePaintTask.action
│   ├── CMakeLists.txt
│   └── package.xml
│
└── wall_paint_description/    # URDF：四輪車 + UR10
    ├── urdf/
    │   ├── mobile_base.xacro     # 四輪車底盤
    │   ├── paint_robot.xacro     # 組合：底盤 + UR10 + 噴嘴
    │   └── spray_nozzle.xacro    # 噴嘴末端執行器
    ├── meshes/
    ├── config/
    │   └── ros2_controllers.yaml
    ├── CMakeLists.txt
    └── package.xml
```
