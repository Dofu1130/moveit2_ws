// 引入 MoveIt Task Constructor (MTC) 相關標頭檔
#include <moveit/task_constructor/solvers.h> // 路徑規劃求解器
#include <moveit/task_constructor/stages.h>  // 任務階段定義
#include <moveit/task_constructor/task.h>    // 任務主類別

// 引入規劃場景相關標頭檔
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp> // ROS2 C++ 函式庫

// 條件式引入 tf2 轉換工具（相容不同版本）
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// 建立日誌記錄器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// 定義命名空間別名，簡化程式碼
namespace mtc = moveit::task_constructor;

// MTC 任務節點類別
// 負責建立規劃場景、定義任務、規劃並執行機械臂動作
class MTCTaskNode {
    public:
        // 建構函數
        MTCTaskNode(const rclcpp::NodeOptions& options);

        // 取得節點基底介面（用於加入執行器）
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
        getNodeBaseInterface();

        // 執行任務（規劃 + 執行）
        void doTask();

        // 設定規劃場景（加入碰撞物件）
        void setupPlanningScene();

    private:
        // Compose an MTC task from a series of stages.
        // 建立由多個階段組成的 MTC 任務
        mtc::Task createTask();
        mtc::Task task_;               // 任務物件
        rclcpp::Node::SharedPtr node_; // ROS2 節點
};

// 建構函數：建立名為 "mtc_node" 的 ROS2 節點
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)} {}

// 回傳節點基底介面，讓執行器可以加入此節點
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCTaskNode::getNodeBaseInterface() {
    return node_->get_node_base_interface();
}

// 設定規劃場景：在場景中加入一個圓柱體作為要抓取的物件
void MTCTaskNode::setupPlanningScene() {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";             // 物件 ID
    object.header.frame_id = "world"; // 參考座標系
    object.primitives.resize(1);
    object.primitives[0].type =
        shape_msgs::msg::SolidPrimitive::CYLINDER; // 圓柱體形狀
    object.primitives[0].dimensions = {0.1, 0.02}; // 高度 0.1m, 半徑 0.02m

    // 設定物件位置
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;    // X 座標
    pose.position.y = -0.25;  // Y 座標
    pose.orientation.w = 1.0; // 姿態（四元數）
    object.pose = pose;

    // 將物件加入規劃場景
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

// 執行任務：初始化 -> 規劃 -> 執行
void MTCTaskNode::doTask() {
    task_ = createTask(); // 建立任務

    // 初始化任務，如果失敗則輸出錯誤並返回
    try {
        task_.init();
    } catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    // 嘗試規劃任務（最多嘗試 5 次）
    if (!task_.plan(5)) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    // 發布規劃結果到 RViz 供視覺化檢視
    task_.introspection().publishSolution(*task_.solutions().front());

    // 執行規劃好的任務
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

// 建立 MTC 任務：定義機械臂要執行的各個階段
mtc::Task MTCTaskNode::createTask() {
    mtc::Task task;
    task.stages()->setName("demo task"); // 設定任務名稱
    task.loadRobotModel(node_);          // 載入機器人模型

    // 定義機械臂和夾爪的群組名稱
    const auto& arm_group_name = "panda_arm"; // 機械臂群組
    const auto& hand_group_name = "hand";     // 夾爪群組
    const auto& hand_frame = "panda_hand";    // 夾爪座標系

    // Set task properties
    // 設定任務屬性
    task.setProperty("group", arm_group_name); // 主要控制的群組
    task.setProperty("eef", hand_group_name);  // 末端執行器（End Effector）
    task.setProperty("ik_frame", hand_frame);  // 逆運動學參考座標系

// Disable warnings for this line, as it's a variable that's set but not used in
// this example
// 暫時關閉編譯器警告（這個變數在此範例中沒有用到，但在完整版本中會用到）
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr =
        nullptr; // Forward current_state on to grasp pose generator
                 // 用於將當前狀態傳遞給抓取姿態生成器
#pragma GCC diagnostic pop

    // 階段 1：取得當前狀態
    // 這是任務的起始點，記錄機械臂目前的位置
    auto stage_state_current =
        std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // 建立不同類型的路徑規劃器
    // Pipeline 規劃器：使用 OMPL 等規劃演算法
    auto sampling_planner =
        std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    // 關節插值規劃器：簡單的關節空間插值
    auto interpolation_planner =
        std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // 笛卡爾路徑規劃器：沿直線移動末端執行器
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);     // 最大速度比例
    cartesian_planner->setMaxAccelerationScalingFactor(1.0); // 最大加速度比例
    cartesian_planner->setStepSize(.01);                     // 步進大小 0.01m

    // 階段 2：打開夾爪
    // 使用關節插值規劃器將夾爪移動到 "open" 姿態
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>(
        "open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name); // 設定要控制的群組
    stage_open_hand->setGoal("open");           // 目標姿態名稱
    task.add(std::move(stage_open_hand));       // 將階段加入任務

    // 階段 3：移動到抓取位置（Connector 類型）
    // Connect 會連接「打開夾爪後的狀態」和「抓取位置」，自動規劃中間的路徑
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{
                            {arm_group_name, sampling_planner}});
    stage_move_to_pick->setTimeout(5.0); // 規劃超時時間 5 秒
    stage_move_to_pick->properties().configureInitFrom(
        mtc::Stage::PARENT); // 從父階段繼承屬性
    task.add(std::move(stage_move_to_pick));

    // 用於稍後傳遞給放置姿態生成器（本範例未使用）
    mtc::Stage* attach_object_stage =
        nullptr; // Forward attach_object_stage to place pose generator
                 // 用於將附加物件的階段傳遞給放置姿態生成器

    // 建立一個 SerialContainer（串列容器）來包含抓取相關的所有子階段
    // 這樣可以把多個階段組織在一起，形成一個「抓取」的邏輯單元
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    // 將任務的屬性暴露給這個容器
    task.properties().exposeTo(grasp->properties(),
                               {"eef", "group", "ik_frame"});
    // 從父階段繼承這些屬性
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "group", "ik_frame"});

    // 階段 4：接近物件（Propagator 類型）
    // 使用笛卡爾路徑規劃器，沿直線慢慢靠近物件
    auto stage_approach = std::make_unique<mtc::stages::MoveRelative>(
        "approach object", cartesian_planner);
    stage_approach->properties().set("marker_ns",
                                     "approach_object");  // RViz 標記命名空間
    stage_approach->properties().set("link", hand_frame); // 移動的連結
    stage_approach->properties().configureInitFrom(mtc::Stage::PARENT,
                                                   {"group"});
    stage_approach->setMinMaxDistance(0.1, 0.15); // 移動距離：10-15 公分

    // Set hand forward direction
    // 設定移動方向：沿夾爪的 Z 軸（向前）
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;
    stage_approach->setDirection(vec);
    grasp->insert(std::move(stage_approach));

    // 階段 5：生成抓取姿態（Generator 類型）
    // 這是一個 Generator！會繞著物件生成多個可能的抓取角度
    auto stage_generate_grasp =
        std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage_generate_grasp->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_generate_grasp->properties().set("marker_ns",
                                           "grasp_pose"); // RViz 標記命名空間
    stage_generate_grasp->setPreGraspPose("open");        // 抓取前的姿態
    stage_generate_grasp->setObject("object");            // 要抓取的物件名稱
    stage_generate_grasp->setAngleDelta(M_PI / 12); // 每 15 度生成一個抓取姿態
    stage_generate_grasp->setMonitoredStage(
        current_state_ptr); // Hook into current state
                            // 監控當前狀態，確保抓取姿態可達

    // 定義抓取座標系的變換（旋轉 + 平移）
    // 這決定了夾爪相對於物件的姿態
    Eigen::Isometry3d grasp_frame_transform;
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(M_PI / 2,
                          Eigen::Vector3d::UnitX()) * // 繞 X 軸旋轉 90 度
        Eigen::AngleAxisd(M_PI / 2,
                          Eigen::Vector3d::UnitY()) * // 繞 Y 軸旋轉 90 度
        Eigen::AngleAxisd(M_PI / 2,
                          Eigen::Vector3d::UnitZ()); // 繞 Z 軸旋轉 90 度
    grasp_frame_transform.linear() = q.matrix();
    grasp_frame_transform.translation().z() = 0.1; // 沿 Z 軸偏移 0.1m

    // Compute IK
    // 計算逆運動學（IK）：將末端執行器的姿態轉換成關節角度
    auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
        "grasp pose IK", std::move(stage_generate_grasp));
    wrapper->setMaxIKSolutions(8);        // 最多計算 8 個 IK 解
    wrapper->setMinSolutionDistance(1.0); // 解之間的最小距離
    wrapper->setIKFrame(grasp_frame_transform, hand_frame); // 設定 IK 座標系
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT,
                                            {"eef", "group"});
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
                                            {"target_pose"});
    grasp->insert(std::move(wrapper));

    // 階段 6：允許碰撞（Propagator 類型）
    // 修改規劃場景，允許夾爪和物件之間的碰撞
    // 否則 MoveIt 會認為夾爪碰到物件是錯誤
    auto stage_allow_collision =
        std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow collision (hand,object)");
    stage_allow_collision->allowCollisions(
        "object",
        task.getRobotModel()
            ->getJointModelGroup(hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry(),
        true); // true = 允許碰撞
    grasp->insert(std::move(stage_allow_collision));

    // 階段 7：關閉夾爪（Propagator 類型）
    // 夾住物件
    auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>(
        "close hand", interpolation_planner);
    stage_close_hand->setGroup(hand_group_name);
    stage_close_hand->setGoal("close"); // 目標姿態：關閉
    grasp->insert(std::move(stage_close_hand));

    // 階段 8：附加物件（Propagator 類型）
    // 把物件「黏」到夾爪上，之後移動夾爪時物件會跟著動
    auto stage_attach =
        std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage_attach->attachObject("object", hand_frame); // 將物件附加到夾爪座標系
    attach_object_stage = stage_attach.get();
    grasp->insert(std::move(stage_attach));

    // 階段 9：抬起物件（Propagator 類型）
    // 使用笛卡爾路徑規劃器，沿直線向上抬起物件
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>(
        "lift object", cartesian_planner);
    stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_lift->setMinMaxDistance(0.1, 0.3); // 抬起距離：10-30 公分
    stage_lift->setIKFrame(hand_frame);
    stage_lift->properties().set("marker_ns", "lift_object");

    // Set upward direction
    // 設定移動方向：沿世界座標的 Z 軸（向上）
    geometry_msgs::msg::Vector3Stamped vec_lift;
    vec_lift.header.frame_id = "world";
    vec_lift.vector.z = 1.0;
    stage_lift->setDirection(vec_lift);
    grasp->insert(std::move(stage_lift));

    // 將整個抓取容器加入任務
    task.add(std::move(grasp));

    return task;
}

// 主程式入口
int main(int argc, char** argv) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 設定節點選項，自動從參數覆蓋中宣告參數
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    // 建立 MTC 任務節點
    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    // 使用多線程執行器（MTC 需要多線程來處理 action 回呼）
    rclcpp::executors::MultiThreadedExecutor executor;

    // 在獨立線程中啟動執行器
    auto spin_thread =
        std::make_unique<std::thread>([&executor, &mtc_task_node]() {
            executor.add_node(mtc_task_node->getNodeBaseInterface());
            executor.spin(); // 開始處理 ROS2 訊息
            executor.remove_node(mtc_task_node->getNodeBaseInterface());
        });

    // 設定規劃場景（加入物件）
    mtc_task_node->setupPlanningScene();
    // 執行任務（規劃 + 執行）
    mtc_task_node->doTask();

    // 等待執行器線程結束
    spin_thread->join();
    // 關閉 ROS2
    rclcpp::shutdown();
    return 0;
}