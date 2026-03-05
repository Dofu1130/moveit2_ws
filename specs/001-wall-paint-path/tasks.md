# Tasks: 牆面噴漆路徑規劃

**Input**: Design documents from `/specs/001-wall-paint-path/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/ros2-interfaces.md

**Tests**: 未在規格中明確要求 TDD，故測試任務標記為選擇性。

**Organization**: 任務依使用者故事分組，每個故事可獨立實作與測試。

## Format: `[ID] [P?] [Story] Description`

- **[P]**: 可並行執行（不同檔案、無相依性）
- **[Story]**: 任務所屬的使用者故事（US1, US2, US3）
- 包含確切的檔案路徑

---

## Phase 1: Setup（專案初始化）

**Purpose**: 安裝相依套件、建立 4 個 ROS2 套件骨架

- [ ] T001 安裝 apt 相依套件：`sudo apt install ros-jazzy-ur ros-jazzy-moveit ros-jazzy-ros2-controllers ros-jazzy-ros-gz`
- [ ] T002 [P] 建立 wall_paint_interfaces 套件骨架（CMakeLists.txt + package.xml）in `src/wall_paint_interfaces/`
- [ ] T003 [P] 建立 wall_paint_description 套件骨架（CMakeLists.txt + package.xml）in `src/wall_paint_description/`
- [ ] T004 [P] 建立 wall_paint_planner 套件骨架（CMakeLists.txt + package.xml）in `src/wall_paint_planner/`
- [ ] T005 [P] 建立 wall_paint_coordinator 套件骨架（CMakeLists.txt + package.xml）in `src/wall_paint_coordinator/`
- [ ] T006 驗證 `colcon build --packages-select wall_paint_interfaces wall_paint_description wall_paint_planner wall_paint_coordinator` 成功

---

## Phase 2: Foundational（基礎建設）

**Purpose**: 定義 ROS2 介面與機器人描述，所有使用者故事的前置條件

**⚠️ CRITICAL**: 此階段完成前不可開始任何使用者故事的實作

### ROS2 介面定義

- [ ] T007 [P] 定義 PlanPaintPath.srv（牆面參數輸入、路徑點輸出）in `src/wall_paint_interfaces/srv/PlanPaintPath.srv`
- [ ] T008 [P] 定義 ValidatePath.srv（路徑可達性驗證）in `src/wall_paint_interfaces/srv/ValidatePath.srv`
- [ ] T009 [P] 定義 ExecutePaintTask.action（完整噴漆任務執行與回饋）in `src/wall_paint_interfaces/action/ExecutePaintTask.action`
- [ ] T010 建置 wall_paint_interfaces 並驗證訊息產生成功 `colcon build --packages-select wall_paint_interfaces`

### 機器人描述（URDF/Xacro）

- [ ] T011 建立四輪車底盤 xacro（4 輪 + diff_drive ros2_control 介面）in `src/wall_paint_description/urdf/mobile_base.xacro`
- [ ] T012 [P] 建立噴嘴末端執行器 xacro in `src/wall_paint_description/urdf/spray_nozzle.xacro`
- [ ] T013 建立組合 xacro（底盤 + UR10 include + 噴嘴，固定關節連接）in `src/wall_paint_description/urdf/paint_robot.xacro`
- [ ] T014 [P] 設定 ros2_controllers.yaml（diff_drive_controller + joint_trajectory_controller + joint_state_broadcaster）in `src/wall_paint_description/config/ros2_controllers.yaml`
- [ ] T015 [P] 設定 kinematics.yaml（UR10 IK 求解器）in `src/wall_paint_description/config/kinematics.yaml`
- [ ] T016 [P] 設定 pilz_cartesian_limits.yaml（笛卡爾速度/加速度限制）in `src/wall_paint_description/config/pilz_cartesian_limits.yaml`
- [ ] T017 建立 display.launch.py（RViz 預覽 URDF）in `src/wall_paint_description/launch/display.launch.py`
- [ ] T018 驗證 URDF 載入 RViz 正確顯示：四輪車底盤 + UR10 手臂 + 噴嘴

**Checkpoint**: 基礎建設完成 — 介面定義已產生、URDF 載入正常，可開始使用者故事實作

---

## Phase 3: User Story 1 — 平面牆壁自動噴漆覆蓋 (Priority: P1) 🎯 MVP

**Goal**: 給定固定牆面參數，系統產生 zig-zag 掃描路徑，機械手臂按路徑執行動作，四輪車在條帶間平移

**Independent Test**: 以硬編碼牆面參數（3m x 2.5m）啟動 demo launch，確認機械手臂在模擬中完成 zig-zag 掃描路徑

### 路徑規劃實作

- [ ] T019 [P] [US1] 定義 PathGenerator 類別標頭（zig-zag 路徑產生介面）in `src/wall_paint_planner/include/wall_paint_planner/path_generator.hpp`
- [ ] T020 [P] [US1] 定義 PathValidator 類別標頭（IK 可達性驗證介面）in `src/wall_paint_planner/include/wall_paint_planner/path_validator.hpp`
- [ ] T021 [US1] 實作 zig-zag 路徑產生演算法（輸入牆面寬高，輸出 PoseArray 路徑點，含條帶重疊 10%-20%）in `src/wall_paint_planner/src/path_generator.cpp`
- [ ] T022 [US1] 實作 IK 可達性驗證（透過 MoveIt2 MoveGroupInterface 檢查每個路徑點）in `src/wall_paint_planner/src/path_validator.cpp`
- [ ] T023 [US1] 實作路徑規劃 ROS2 節點（PlanPaintPath service server + ValidatePath service server）in `src/wall_paint_planner/src/planner_node.cpp`
- [ ] T024 [US1] 建立規劃器參數設定檔（spray_width、strip_overlap、scan_speed、wall_distance、arm_reach）in `src/wall_paint_planner/config/planner_params.yaml`

### 任務協調與底盤控制

- [ ] T025 [P] [US1] 定義 BaseController 類別標頭（須透過介面抽象底盤移動操作，使測試時可替換為 mock）in `src/wall_paint_coordinator/include/wall_paint_coordinator/base_controller.hpp`
- [ ] T026 [P] [US1] 定義 TaskCoordinator 類別標頭（須透過介面抽象手臂控制和底盤控制的依賴注入，使測試時可替換為 mock）in `src/wall_paint_coordinator/include/wall_paint_coordinator/task_coordinator.hpp`
- [ ] T027 [US1] 實作 BaseController（實作底盤移動介面，發送 cmd_vel 指令使底盤左右平移固定距離，等待 odom 回饋確認到位，含 position_tolerance 容差參數與超時處理）in `src/wall_paint_coordinator/src/base_controller.cpp`
- [ ] T028 [US1] 實作 TaskCoordinator（順序執行迴圈：呼叫路徑規劃 → Pilz LIN 執行手臂掃描 → 底盤平移 → 重複直到所有條帶完成；須訂閱 /emergency_stop topic，收到時立即停止手臂與底盤動作）in `src/wall_paint_coordinator/src/task_coordinator.cpp`

### 單元測試（憲章原則 II：可測試性）

- [ ] T029 [P] [US1] 撰寫 PathGenerator 單元測試（脫離 ROS2 環境，驗證 zig-zag 路徑點數量、覆蓋率、條帶重疊率）in `src/wall_paint_planner/test/test_path_generator.cpp`
- [ ] T030 [P] [US1] 撰寫 PathValidator 單元測試（注入 mock IK solver，驗證可達性判斷邏輯）in `src/wall_paint_planner/test/test_path_validator.cpp`

### 整合與驗證

- [ ] T031 [US1] 建立 wall_paint_demo.launch.py（啟動 UR10 MoveIt2 + planner_node + task_coordinator，使用硬編碼 3m x 2.5m 牆面）in `src/wall_paint_planner/launch/wall_paint_demo.launch.py`
- [ ] T032 [US1] 建置並驗證 MVP：執行 demo launch，確認 zig-zag 路徑在 RViz 中顯示且手臂按路徑移動

**Checkpoint**: US1 完成 — 以固定參數可展示完整的噴漆路徑規劃與執行流程

---

## Phase 4: User Story 2 — 牆壁尺寸定義與任務設定 (Priority: P2)

**Goal**: 操作員可透過 ROS2 service 動態輸入牆壁寬高，系統自動計算並執行對應路徑

**Independent Test**: 以 `ros2 service call` 傳入不同牆面尺寸，驗證路徑點數量和條帶數正確

### 實作

- [ ] T033 [US2] 擴充 PathGenerator 加入牆面尺寸驗證邏輯（範圍檢查、超出最大覆蓋範圍時回傳錯誤，統一處理 FR-008）in `src/wall_paint_planner/src/path_generator.cpp`
- [ ] T034 [US2] 擴充 planner_node 轉發 PathGenerator 的驗證錯誤為 service response in `src/wall_paint_planner/src/planner_node.cpp`
- [ ] T035 [US2] 擴充 TaskCoordinator 改為透過 ExecutePaintTask action 接收動態牆面參數（取代硬編碼）in `src/wall_paint_coordinator/src/task_coordinator.cpp`
- [ ] T036 [US2] 建立 coordinator.launch.py（啟動協調器並暴露 action server 介面）in `src/wall_paint_coordinator/launch/coordinator.launch.py`
- [ ] T037 [US2] 驗證：以 `ros2 action send_goal` 傳入不同牆面尺寸（1m x 1m、3m x 2.5m、超大尺寸），確認路徑正確產生或警告觸發

**Checkpoint**: US2 完成 — 操作員可動態輸入牆面參數，系統自動計算路徑

---

## Phase 5: User Story 3 — 模擬環境中驗證路徑 (Priority: P3)

**Goal**: 操作員在 Gazebo + RViz 中預覽完整路徑，系統顯示碰撞警告

**Independent Test**: 在 Gazebo 中載入牆壁模型，執行路徑規劃，確認 RViz 中路徑軌跡可視化且碰撞偵測正常

### 實作

- [ ] T038 [P] [US3] 建立 Gazebo 世界檔案（含牆壁模型）in `src/wall_paint_description/worlds/paint_wall.sdf`
- [ ] T039 [P] [US3] 在 planner_node 中加入 RViz Marker 發佈（顯示規劃路徑軌跡、條帶分界線、覆蓋區域）in `src/wall_paint_planner/src/planner_node.cpp`
- [ ] T040 [US3] 在 path_validator 中加入碰撞偵測結果視覺化回饋（標記不可達/碰撞路徑點為紅色 Marker）in `src/wall_paint_planner/src/path_validator.cpp`
- [ ] T041 [US3] 建立完整模擬 launch 檔案（Gazebo Harmonic + ros_gz bridge + UR10 MoveIt2 + planner + coordinator）in `src/wall_paint_description/launch/simulation.launch.py`
- [ ] T042 [US3] 驗證：啟動模擬環境，執行路徑規劃，確認 RViz 中可看到完整路徑軌跡且碰撞警告正常顯示

**Checkpoint**: US3 完成 — 操作員可在模擬環境中完整預覽噴漆路徑

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: 跨故事改善與邊界情況處理

- [ ] T043 [P] 處理邊界情況：手臂接近關節限制時的路徑調整策略 in `src/wall_paint_planner/src/path_validator.cpp`
- [ ] T044 [P] 效能量測：驗證 3m x 2.5m 牆面路徑規劃耗時 < 10 秒（SC-001），記錄規劃時間至 log
- [ ] T045 驗證 `colcon build --symlink-install` 零警告通過
- [ ] T046 執行 quickstart.md 完整流程驗證（從安裝到 demo 執行）

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: 無相依性 — 立即開始
- **Foundational (Phase 2)**: 依賴 Phase 1 完成 — **阻擋所有使用者故事**
- **US1 (Phase 3)**: 依賴 Phase 2 完成 — **MVP 核心**
- **US2 (Phase 4)**: 依賴 Phase 2 完成，建議在 US1 後執行（擴充同一批檔案）
- **US3 (Phase 5)**: 依賴 Phase 2 完成，建議在 US1 後執行（需要路徑規劃結果來視覺化）
- **Polish (Phase 6)**: 依賴所有使用者故事完成

### User Story Dependencies

- **US1 (P1)**: Phase 2 完成即可開始，不依賴其他故事
- **US2 (P2)**: 理論上可與 US1 並行，但實際上修改相同檔案（planner_node.cpp、task_coordinator.cpp），建議循序執行
- **US3 (P3)**: 可與 US1/US2 並行（Gazebo 世界檔案和 Marker 發佈為獨立檔案），但碰撞視覺化需要 path_validator 已實作

### Within Each User Story

- 標頭檔（.hpp）先於實作檔（.cpp）
- 路徑產生先於路徑驗證
- 核心邏輯先於 ROS2 節點包裝
- 節點實作先於 launch 檔案
- 整合驗證為最後步驟

### Parallel Opportunities

**Phase 1 並行**:
```
T002 (interfaces) ‖ T003 (description) ‖ T004 (planner) ‖ T005 (coordinator)
```

**Phase 2 並行**:
```
T007 (PlanPaintPath.srv) ‖ T008 (ValidatePath.srv) ‖ T009 (ExecutePaintTask.action)
T014 (controllers.yaml) ‖ T015 (kinematics.yaml) ‖ T016 (pilz_limits.yaml)
```

**Phase 3 並行**:
```
T019 (path_generator.hpp) ‖ T020 (path_validator.hpp)
T025 (base_controller.hpp) ‖ T026 (task_coordinator.hpp)
T029 (test_path_generator) ‖ T030 (test_path_validator)
```

**Phase 5 並行**:
```
T038 (Gazebo world) ‖ T039 (RViz markers)
```

---

## Implementation Strategy

### MVP First（僅 User Story 1）

1. 完成 Phase 1: Setup
2. 完成 Phase 2: Foundational（**關鍵阻擋點**）
3. 完成 Phase 3: User Story 1
4. **停下來驗證**: 以硬編碼參數執行 demo，確認 zig-zag 路徑在模擬中正常運作
5. 可立即展示 MVP

### 增量交付

1. Setup + Foundational → 基礎就緒
2. +US1 → 獨立測試 → 展示 MVP（核心噴漆路徑）
3. +US2 → 獨立測試 → 展示（動態牆面參數輸入）
4. +US3 → 獨立測試 → 展示（完整模擬預覽）
5. 每個故事都為前一個增加價值，不破壞已有功能

---

## Notes

- [P] 任務 = 不同檔案、無相依性，可並行
- [Story] 標籤對應 spec.md 中的使用者故事
- 每個使用者故事可獨立完成與測試
- 每完成一個任務或邏輯群組後 commit
- 在每個 Checkpoint 暫停驗證功能
