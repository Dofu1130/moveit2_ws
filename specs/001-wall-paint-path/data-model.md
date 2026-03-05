# 資料模型：牆面噴漆路徑規劃

**功能分支**：`001-wall-paint-path`
**日期**：2026-03-05

## 實體定義

### WallSurface（牆面）

待噴漆的平面目標。Phase 1 僅支援無深度的平整矩形牆壁。

| 欄位 | 型別 | 說明 | 驗證規則 |
|------|------|------|----------|
| width | float64 | 牆壁寬度（公尺） | > 0, ≤ 20.0 |
| height | float64 | 牆壁高度（公尺） | > 0, ≤ 5.0 |
| origin | Pose | 牆壁左下角在世界座標系的位置與方向 | 有效的 pose |
| normal | Vector3 | 牆面法向量（指向機器人方向） | 單位向量 |

### PaintStrip（噴漆條帶）

機械手臂一次垂直掃描所覆蓋的區域。

| 欄位 | 型別 | 說明 | 驗證規則 |
|------|------|------|----------|
| strip_index | uint32 | 條帶編號（從左至右） | ≥ 0 |
| x_position | float64 | 條帶中心在牆面座標系的 X 位置 | 在牆面範圍內 |
| strip_width | float64 | 條帶寬度（由噴嘴扇面決定） | > 0 |
| overlap_ratio | float64 | 與前一條帶的重疊比例 | 0.1 ~ 0.2 |
| waypoints | Pose[] | 此條帶的路徑點序列 | 非空，所有點在手臂可達範圍內 |

### ScanPath（掃描路徑）

機械手臂在單一條帶內的上下運動軌跡。

| 欄位 | 型別 | 說明 | 驗證規則 |
|------|------|------|----------|
| direction | enum | UP 或 DOWN（掃描方向） | 有效值 |
| start_pose | Pose | 起始位姿 | 在手臂可達範圍內 |
| end_pose | Pose | 結束位姿 | 在手臂可達範圍內 |
| line_spacing | float64 | 水平間距（公尺） | > 0 |
| speed | float64 | 掃描速度（m/s） | > 0, ≤ 手臂最大速度 |

### ShiftCommand（平移指令）

四輪車在條帶之間的移動指令。

| 欄位 | 型別 | 說明 | 驗證規則 |
|------|------|------|----------|
| distance | float64 | 平移距離（公尺） | > 0 |
| direction | enum | LEFT 或 RIGHT | 有效值 |
| target_x | float64 | 目標位置 | 在牆面覆蓋範圍內 |

### PaintTask（噴漆任務）

一次完整噴漆工作的描述。

| 欄位 | 型別 | 說明 | 驗證規則 |
|------|------|------|----------|
| task_id | string | 任務唯一識別碼 | 非空 |
| wall | WallSurface | 目標牆面 | 有效牆面 |
| strips | PaintStrip[] | 所有噴漆條帶 | 非空 |
| total_strips | uint32 | 條帶總數 | > 0 |
| estimated_coverage | float64 | 預估覆蓋率（%） | 0 ~ 100 |
| status | enum | PLANNED / EXECUTING / COMPLETED / FAILED | 有效狀態 |

## 實體關係

```
PaintTask (1) ──contains──> (1) WallSurface
PaintTask (1) ──contains──> (N) PaintStrip
PaintStrip (1) ──contains──> (1) ScanPath
PaintStrip (N) ──separated by──> (N-1) ShiftCommand
```

## 狀態轉換

### PaintTask 狀態機

```
PLANNED → EXECUTING → COMPLETED
   │          │
   │          └→ FAILED
   └→ (取消/刪除)
```

- **PLANNED → EXECUTING**：操作員確認開始執行
- **EXECUTING → COMPLETED**：所有條帶執行完畢
- **EXECUTING → FAILED**：碰撞偵測觸發、關節超限、或操作員緊急停止

## ROS2 訊息對應

以上實體將對應至以下 ROS2 概念：

- **WallSurface** → 自訂 ROS2 msg 或 ROS2 Parameter
- **PaintStrip / ScanPath** → `geometry_msgs/PoseArray` + 自訂 metadata
- **ShiftCommand** → `geometry_msgs/Twist`（底盤控制）
- **PaintTask** → ROS2 Action（長時間執行的任務）
