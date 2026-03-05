# 研究報告：牆面噴漆路徑規劃

**日期**：2026-03-05
**功能分支**：`001-wall-paint-path`

## 決策 1：UR10 ROS2 Jazzy 支援

**決策**：使用 `ros-jazzy-ur` apt 套件安裝 UR10 驅動與 MoveIt2 設定

**理由**：
- UR10 驅動 `ur_robot_driver` v3.7.0 已正式支援 ROS2 Jazzy
- `ros-jazzy-ur` meta-package 包含所有必要套件：ur_robot_driver、ur_moveit_config、ur_controllers、ur_calibration、ur_dashboard_msgs
- 啟動指令：`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true`

**替代方案**：
- 從原始碼編譯 Jazzy 分支 — 不必要，apt 套件已可用
- 使用 Kinova Gen3（工作區現有） — 工作範圍不足（~0.9m vs UR10 ~1.3m），不適合牆面噴漆

**Jazzy 遷移注意事項**：
- `keep_alive_count` 參數改為 `robot_receive_timeout`
- `<ros2_control>` 標籤由驅動套件生成，非 description 套件
- 機器人描述改由 `robot_state_publisher` topic 發布
- 啟動檔案須使用絕對路徑

## 決策 2：表面覆蓋路徑產生方式

**決策**：Phase 1 使用自訂 zig-zag 路徑點產生器（針對已知平面），不引入 Noether

**理由**：
- Phase 1 目標僅為平整牆壁（無深度），幾何計算簡單
- 定義平面參數（原點、法向量、寬度、高度）後，以固定 Y 間距產生左右交替的掃描路徑點
- 每個路徑點為 `geometry_msgs::Pose`，位置在牆面上，方向與法向量對齊
- 避免引入額外相依性，符合簡約設計原則

**替代方案**：
- Noether（ROS-Industrial 工具路徑規劃器） — Phase 2+ 處理複雜曲面時再引入，Phase 1 不需要
- scan_n_plan_workshop — 包含感知管線，Phase 1 不需要影像辨識

## 決策 3：笛卡爾路徑執行方式

**決策**：使用 MoveIt2 內建的 Pilz Industrial Motion Planner 執行路徑點

**理由**：
- Pilz 支援 LIN（直線）、PTP（點對點）、CIRC（圓弧）運動指令
- 支援 `MotionSequenceRequest` 串接多個路徑點並設定混合半徑，實現平滑轉換
- 確定性規劃器（非取樣式），路徑可預測
- 已內建於 MoveIt2，無須額外安裝

**替代方案**：
- `computeCartesianPath()` — 貪婪演算法，容易陷入區域最小值，無避障能力，PickNik 不建議使用
- Descartes（Tesseract） — 適合欠約束路徑，但需額外引入 Tesseract 框架，Phase 1 過度
- MoveIt Servo — 適合即時遙控，不適合批次路徑點執行

## 決策 4：四輪車移動控制

**決策**：使用 `diff_drive_controller`（差速驅動控制器），採用順序協調策略

**理由**：
- Phase 1 四輪車僅需左右平移，差速驅動即可滿足
- 順序協調策略最簡單可靠：移動底盤 → 停止 → 執行手臂動作 → 重複
- `diff_drive_controller` 已包含在 `ros2_controllers`，ROS2 Jazzy 原生支援

**替代方案**：
- Ackermann 轉向控制器 — 適合類車輛底盤，但 Phase 1 僅需簡單平移
- Nav2 完整導航堆疊 — Phase 1 不需要 SLAM/定位，過度複雜
- 全身運動規劃（Whole-Body Planning） — 將底盤和手臂作為一個規劃群組，MoveIt2 支援有限且複雜

## 決策 5：模擬環境

**決策**：使用 Gazebo Harmonic（ROS2 Jazzy 官方配對模擬器）

**理由**：
- Gazebo Harmonic 是 ROS2 Jazzy 的官方配對版本
- 透過 `ros-jazzy-ros-gz` 整合套件連接 ROS2 和 Gazebo
- UR10 的 Gazebo 模擬已有社群範例可參考

**替代方案**：
- 僅使用 RViz 假動作 — 無物理模擬，無法驗證碰撞和動力學
- 純 MoveIt2 模擬（fake controller） — 可用於初步路徑驗證，但缺乏環境互動

## 決策 6：URDF 架構

**決策**：四輪車底盤 + UR10 手臂透過固定關節連接，使用兩個獨立的 `<ros2_control>` 標籤

**理由**：
- ros2_control 文件確認「移動底盤加手臂的機器人無需額外程式碼，僅需控制器設定檔」
- URDF 結構：`base_link → arm_mount_link（固定） → ur10_base_link → ... → tool0`
- 底盤和手臂各有獨立的硬體介面和控制器

## 新增套件清單

| 套件 | 安裝方式 | 理由 |
|------|----------|------|
| `ros-jazzy-ur` | apt | UR10 驅動、MoveIt config、控制器 |
| `ros-jazzy-ros2-controllers` | apt（已有） | diff_drive_controller 用於底盤控制 |
| `ros-jazzy-ros-gz` | apt | Gazebo Harmonic 模擬整合 |

**注意**：以上均為 apt 安裝的標準 ROS2 Jazzy 套件，不需要從原始碼編譯。
