# Implementation Plan: 牆面噴漆路徑規劃

**Branch**: `001-wall-paint-path` | **Date**: 2026-03-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-wall-paint-path/spec.md`

## 摘要

實作一個 ROS2 Jazzy + MoveIt2 系統，讓 UR10 機械手臂搭載在四輪車上，以 zig-zag 掃描路徑自動填滿平面牆壁。系統分為路徑規劃（產生路徑點）、路徑執行（Pilz LIN 笛卡爾控制）、底盤平移（diff_drive 控制器）、以及任務協調（順序執行策略）四個層次。

## Technical Context

**Language/Version**: C++17（核心節點）、Python 3.x（launch 檔案、工具腳本）
**Primary Dependencies**: ROS2 Jazzy、MoveIt2（含 Pilz Industrial Motion Planner）、ur_robot_driver 3.7.0、ros2_controllers（diff_drive_controller）
**Storage**: N/A（無持久化儲存需求）
**Testing**: colcon test、GTest（C++ 單元測試）、launch_testing（整合測試）
**Target Platform**: Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic
**Project Type**: ROS2 workspace（多套件）
**Performance Goals**: 3m x 2.5m 牆面路徑規劃 < 10 秒、覆蓋率 ≥ 95%
**Constraints**: 所有路徑點在 UR10 可達範圍內、碰撞檢測通過率 100%
**Scale/Scope**: 4 個 ROS2 套件、Phase 1 MVP 僅支援平面牆壁

## 憲章合規檢查（Constitution Check）

*GATE: 必須在 Phase 0 研究前通過。Phase 1 設計完成後再次確認。*

| 閘門 | 檢查項目 | 狀態 |
|------|----------|------|
| **閘門 1（可測試性）** | 每個使用者故事是否有獨立的 Given/When/Then 驗收場景？ | ☑ 通過 — P1/P2/P3 各有 2-3 個驗收場景 |
| **閘門 2（MVP 優先）** | P1 故事是否能單獨交付可展示的成果，不依賴 P2/P3？ | ☑ 通過 — P1 僅需固定牆面參數即可展示路徑規劃與執行 |
| **閘門 3（簡約設計）** | 是否存在未在 Complexity Tracking 說明的額外抽象或設計模式？ | ☑ 通過 — 無額外抽象，使用標準 ROS2 節點/服務/動作模式 |
| **閘門 4（技術堆疊）** | 計畫是否只使用工作區已有的依賴？若需新增，是否已說明理由？ | ☑ 通過（附說明） — 新增 UR10 驅動與 Gazebo，詳見 Complexity Tracking |

## Project Structure

### Documentation (this feature)

```text
specs/001-wall-paint-path/
├── spec.md              # 功能規格
├── plan.md              # 本文件
├── research.md          # Phase 0 研究報告
├── data-model.md        # 資料模型
├── quickstart.md        # 快速開始指南
├── contracts/
│   └── ros2-interfaces.md  # ROS2 介面合約
└── checklists/
    └── requirements.md  # 規格品質檢查表
```

### Source Code (repository root)

```text
src/
├── wall_paint_planner/        # 路徑規劃套件
│   ├── src/
│   │   ├── path_generator.cpp    # zig-zag 路徑點產生
│   │   ├── path_validator.cpp    # IK 可達性驗證
│   │   └── planner_node.cpp      # ROS2 節點（Service Server）
│   ├── include/wall_paint_planner/
│   │   ├── path_generator.hpp
│   │   └── path_validator.hpp
│   ├── launch/
│   │   └── wall_paint_demo.launch.py
│   ├── config/
│   │   └── planner_params.yaml
│   ├── test/
│   │   ├── test_path_generator.cpp   # 單元測試
│   │   └── test_path_validator.cpp   # 單元測試
│   ├── CMakeLists.txt
│   └── package.xml
│
├── wall_paint_coordinator/    # 任務協調套件
│   ├── src/
│   │   ├── task_coordinator.cpp  # 順序執行：底盤移動 → 手臂噴漆 → 重複
│   │   └── base_controller.cpp   # 底盤左右平移控制
│   ├── include/wall_paint_coordinator/
│   │   ├── task_coordinator.hpp
│   │   └── base_controller.hpp
│   ├── launch/
│   │   └── coordinator.launch.py
│   ├── test/
│   │   └── test_base_controller.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
├── wall_paint_interfaces/     # 自訂 ROS2 介面
│   ├── srv/
│   │   ├── PlanPaintPath.srv
│   │   └── ValidatePath.srv
│   ├── action/
│   │   └── ExecutePaintTask.action
│   ├── CMakeLists.txt
│   └── package.xml
│
└── wall_paint_description/    # URDF 描述
    ├── urdf/
    │   ├── mobile_base.xacro     # 四輪車底盤定義
    │   ├── paint_robot.xacro     # 組合：底盤 + UR10 + 噴嘴
    │   └── spray_nozzle.xacro    # 噴嘴末端執行器
    ├── meshes/                   # 3D 模型
    ├── config/
    │   ├── ros2_controllers.yaml # 底盤 + 手臂控制器設定
    │   ├── kinematics.yaml       # IK 求解器設定
    │   └── pilz_cartesian_limits.yaml  # Pilz 笛卡爾限制
    ├── launch/
    │   └── display.launch.py     # RViz 預覽
    ├── CMakeLists.txt
    └── package.xml
```

**Structure Decision**: 採用 ROS2 標準多套件架構，將介面定義、機器人描述、路徑規劃、任務協調分離為獨立套件，各自可獨立測試與建置。

## Complexity Tracking

> 新增技術堆疊說明（閘門 4）

| 變更 | 必要理由 | 未採用的簡單替代方案 |
|------|----------|----------------------|
| 新增 `ros-jazzy-ur`（UR10 驅動） | Spec 指定使用 UR10，工作範圍 1.3m 滿足牆面噴漆需求，工作區現有的 Kinova Gen3 Lite 工作範圍不足 | 繼續使用 Kinova Gen3 Lite — 工作範圍僅 ~0.9m，不足以覆蓋實際噴漆場景 |
| 新增 `ros-jazzy-ros-gz`（Gazebo Harmonic） | 需要物理模擬環境驗證碰撞和路徑執行，憲章安全要求規定「控制指令 MUST 在模擬環境中先驗證」 | 僅用 RViz fake controller — 無物理模擬，無法驗證碰撞 |
| 4 個新 ROS2 套件 | 遵循 ROS2 慣例：介面/描述/邏輯分離。每個套件職責單一，可獨立測試 | 全部寫在一個套件 — 違反單一職責原則，測試困難 |
