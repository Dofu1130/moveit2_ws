# ROS2 介面合約：牆面噴漆路徑規劃

**功能分支**：`001-wall-paint-path`
**日期**：2026-03-05

## ROS2 Topics

### 發佈（Publish）

| Topic 名稱 | 訊息型別 | 發佈者 | 說明 |
|------------|----------|--------|------|
| `/paint_path/waypoints` | `geometry_msgs/PoseArray` | 路徑規劃節點 | 目前條帶的掃描路徑點 |
| `/paint_task/status` | `std_msgs/String` | 任務協調節點 | 任務狀態（PLANNED/EXECUTING/COMPLETED/FAILED） |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盤控制節點 | 四輪車速度指令 |

### 訂閱（Subscribe）

| Topic 名稱 | 訊息型別 | 訂閱者 | 說明 |
|------------|----------|--------|------|
| `/odom` | `nav_msgs/Odometry` | 任務協調節點 | 底盤里程計回饋 |
| `/joint_states` | `sensor_msgs/JointState` | 路徑規劃節點 | 機械手臂關節狀態 |

## ROS2 Services

| Service 名稱 | 型別 | 說明 |
|-------------|------|------|
| `/paint_path/plan` | 自訂 srv（見下方） | 輸入牆面參數，回傳路徑規劃結果 |
| `/paint_path/validate` | 自訂 srv | 驗證路徑是否在手臂可達範圍內 |

### PlanPaintPath.srv

```
# Request
float64 wall_width        # 牆壁寬度（公尺）
float64 wall_height       # 牆壁高度（公尺）
geometry_msgs/Pose wall_origin  # 牆壁左下角位姿
float64 strip_overlap     # 條帶重疊比例（預設 0.15）
---
# Response
bool success
string message
uint32 total_strips       # 條帶總數
geometry_msgs/PoseArray[] strip_waypoints  # 每個條帶的路徑點
float64 estimated_coverage  # 預估覆蓋率（%）
```

### ValidatePath.srv

```
# Request
geometry_msgs/PoseArray waypoints  # 待驗證的路徑點
---
# Response
bool all_reachable
uint32[] unreachable_indices  # 不可達路徑點的索引
bool collision_free
string message
```

## ROS2 Actions

### ExecutePaintTask.action

```
# Goal
float64 wall_width
float64 wall_height
geometry_msgs/Pose wall_origin
float64 strip_overlap
---
# Result
bool success
float64 actual_coverage    # 實際覆蓋率
uint32 strips_completed    # 已完成條帶數
string message
---
# Feedback
uint32 current_strip       # 目前執行的條帶編號
uint32 total_strips        # 條帶總數
string phase               # PLANNING / MOVING_BASE / PAINTING / TRANSITIONING
float64 progress_percent   # 總進度百分比
```

## ROS2 Parameters

### 路徑規劃節點參數

| 參數名稱 | 型別 | 預設值 | 說明 |
|----------|------|--------|------|
| `robot_type` | string | "ur10" | 機器人型號 |
| `spray_width` | double | 0.3 | 噴嘴扇面寬度（公尺） |
| `strip_overlap` | double | 0.15 | 條帶重疊比例 |
| `scan_speed` | double | 0.1 | 掃描速度（m/s） |
| `wall_distance` | double | 0.5 | 噴嘴到牆面的距離（公尺） |
| `arm_reach` | double | 1.3 | 手臂最大工作半徑（公尺） |

### 底盤控制參數

| 參數名稱 | 型別 | 預設值 | 說明 |
|----------|------|--------|------|
| `base_shift_speed` | double | 0.1 | 平移速度（m/s） |
| `position_tolerance` | double | 0.01 | 定位容差（公尺） |

## TF 座標框架

```
world
  └── odom
       └── base_link（四輪車底盤）
            ├── wheel_left_front
            ├── wheel_left_rear
            ├── wheel_right_front
            ├── wheel_right_rear
            └── arm_mount_link（固定關節）
                 └── ur10_base_link
                      └── ... (UR10 joints) ...
                           └── tool0
                                └── spray_nozzle_link
```

## 節點架構

```
[wall_paint_planner_node]  ── 路徑規劃
        │
        ├── Service: /paint_path/plan
        ├── Service: /paint_path/validate
        └── Pub: /paint_path/waypoints

[paint_task_coordinator_node]  ── 任務協調（順序執行）
        │
        ├── Action Server: /execute_paint_task
        ├── 呼叫 MoveIt2 規劃與執行
        ├── 發送 /cmd_vel 控制底盤
        └── Pub: /paint_task/status
```
