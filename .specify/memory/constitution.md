<!--
SYNC IMPACT REPORT
==================
版本變更：1.0.0 → 1.0.1
變更類型：PATCH（技術堆疊澄清：新增 ROS2 Jazzy 與 UR 機器人支援）

新增章節：
  - 核心原則 I–V（全新）
  - 額外約束
  - 開發工作流程
  - 治理規則

修改原則：無（首次定義）
移除原則：無

需要更新的模板：
  ✅ .specify/memory/constitution.md（本檔案）
  ✅ .specify/templates/plan-template.md（Constitution Check 閘門已對齊本憲章原則）
  ✅ .specify/templates/spec-template.md（使用者故事優先順序與 MVP 優先原則一致）
  ✅ .specify/templates/tasks-template.md（任務分組方式與 MVP 增量交付一致）
  ⚠ .specify/templates/agent-file-template.md（僅為骨架模板，待首個 feature 填入後自動更新）

延後事項：無未解析的佔位符。
-->

# MoveIt2 機器人運動規劃工作區 憲章

## 核心原則

### I. 品質優先

每一行程式碼都必須（MUST）達到可維護、可閱讀的水準。
具體要求：

- 程式碼 MUST 遵循語言/框架的慣用風格（C++17 for ROS2/MoveIt2、Python 3.x for 工具腳本）
- 函式與類別 MUST 具有單一明確的職責（Single Responsibility Principle）
- 所有公開 API MUST 附有說明其用途、參數與回傳值的文件
- 重複程式碼超過三處時 MUST 抽取為共用函式，禁止複製貼上
- 提交前 MUST 確認無編譯警告、無 linting 違規

**理由**：機器人控制程式碼的錯誤可能導致硬體損毀或安全事故，品質是不可妥協的底線。

### II. 可測試性（NON-NEGOTIABLE）

每個功能單元都 MUST 能被獨立測試，無論是單元測試、整合測試或手動驗證。
具體要求：

- 每個使用者故事（User Story）MUST 能在不依賴其他故事的情況下獨立驗收
- 邏輯層（service/library）MUST 與 ROS2 節點解耦，以便脫離 ROS 環境進行單元測試
- 有副作用的操作（硬體指令、檔案寫入）MUST 透過介面抽象，允許測試時替換為假物件（mock/stub）
- 驗收場景（Acceptance Scenarios）MUST 使用 Given/When/Then 格式明確描述
- 若有自動化測試，測試 MUST 在實作前撰寫並確認失敗（Red-Green-Refactor）

**理由**：可測試性是品質的可量化證明，也是重構的安全網。

### III. 最小可行產品優先（MVP First）

每個功能 MUST 以最小可交付的形式規劃，先驗證核心價值再擴展。
具體要求：

- 規格（spec.md）MUST 將使用者故事依優先順序排列（P1 → Pn）
- P1 故事本身 MUST 構成一個完整、可展示的 MVP
- 實作 MUST 依優先順序進行，P1 完成並通過驗收後才能開始 P2
- 每個里程碑（Checkpoint）MUST 暫停驗收，確認功能正常後再繼續

**理由**：提早驗證降低浪費，避免在錯誤方向上過度投入。

### IV. 簡約設計（No Overdesign）

在滿足當前需求的前提下，選擇最簡單的解法。
具體要求：

- MUST 遵守 YAGNI（You Aren't Gonna Need It）：禁止為假設的未來需求設計
- 若三處類似程式碼才需要抽象，不得因「可能重用」而提前抽取
- 禁止為單次使用的操作建立 helper/utility class
- 新增的設計模式（Repository、Factory、Event Bus 等）MUST 在 plan.md 的 Complexity Tracking 中說明必要性
- 介面數量 MUST 最小化：能用具體型別解決的問題，不引入介面層

**理由**：在機器人開發的探索階段，過度抽象使程式難以理解、難以除錯，增加維護成本。

### V. 文件正體中文化

本專案所有規格、計畫、任務文件 MUST 以正體中文撰寫。
具體要求：

- `specs/` 目錄下所有 Markdown 文件 MUST 以正體中文撰寫
- 程式碼中的識別字（變數名、函式名、類別名）MUST 使用英文
- 程式碼內嵌的行內註解 SHOULD 使用正體中文，以提高可讀性
- 技術術語（如 MoveIt2、ROS2、MTC）保留英文原文，不強制翻譯
- Commit message 允許中英混用，但主旨（subject line）SHOULD 以中文描述意圖

**理由**：統一語言降低溝通成本，正體中文是團隊的主要工作語言。

## 額外約束

### 技術堆疊

本工作區的核心技術為：

- **運動規劃框架**：MoveIt2（基於 ROS2 Humble/Iron/Jazzy）
- **任務規劃**：MoveIt Task Constructor（MTC）
- **機器人硬體**：Kinova Gen3 Lite（ros2_kortex）、Universal Robots UR5/UR10（ur_robot_driver，噴漆機器人用途）
- **夾爪**：Robotiq 2F 系列（ros2_robotiq_gripper）
- **視覺化**：RViz2 + moveit_visual_tools
- **程式語言**：C++17（核心節點）、Python 3.x（工具腳本、測試）
- **建置系統**：colcon + CMake

### 安全要求

- 所有控制機器人的指令 MUST 在模擬環境（Gazebo/RViz 假動作）中先驗證
- 與真實硬體互動的程式碼 MUST 有明確的緊急停止（emergency stop）路徑
- 關節限制（joint limits）與碰撞物件 MUST 在 MoveIt 設定中正確定義

## 開發工作流程

1. 功能開發 MUST 在獨立的 feature branch 上進行（命名格式：`NNN-功能描述`）
2. 每個 feature MUST 依序完成：`spec.md` → `plan.md` → `tasks.md` → 實作
3. 實作完成後 MUST 在 `main` 分支上可正常建置（`colcon build --symlink-install`）
4. Constitution Check MUST 在 `plan.md` 建立時執行，驗證以下閘門：
   - **閘門 1（可測試性）**：每個使用者故事是否有獨立的驗收場景？
   - **閘門 2（MVP 優先）**：P1 故事是否能單獨交付可展示的成果？
   - **閘門 3（簡約設計）**：是否存在未在 Complexity Tracking 說明的額外抽象？
   - **閘門 4（技術堆疊）**：計畫是否只使用本工作區已有的依賴？若需新增，MUST 說明理由。

## 治理規則

本憲章是所有功能開發的最高準則，優先於個別功能的 spec 與 plan。

**版本控制規則**：
- MAJOR：移除或根本性重新定義原則（破壞性變更）
- MINOR：新增原則或章節
- PATCH：措辭澄清、錯字修正、非語意性調整

**修訂程序**：任何原則修訂 MUST 同步更新本文件版本號、`LAST_AMENDED_DATE`，並檢視
`.specify/templates/` 下所有模板是否需要對應調整。

**合規審查**：每次 `plan.md` 建立時，Constitution Check 章節即作為合規審查的執行記錄。

**版本**：1.0.1 | **批准日期**：2026-03-02 | **最後修訂**：2026-03-05
