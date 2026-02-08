// 引入 MoveIt Task Constructor (MTC) 相關標頭檔
#include <moveit/task_constructor/solvers.h>  // 路徑規劃求解器
#include <moveit/task_constructor/stages.h>   // 任務階段定義
#include <moveit/task_constructor/task.h>     // 任務主類別

// 引入規劃場景相關標頭檔
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>  // ROS2 C++ 函式庫

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
        mtc::Task task_;              // 任務物件
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
    object.id = "object";                    // 物件 ID
    object.header.frame_id = "world";        // 參考座標系
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;  // 圓柱體形狀
    object.primitives[0].dimensions = {0.1, 0.02};  // 高度 0.1m, 半徑 0.02m

    // 設定物件位置
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;      // X 座標
    pose.position.y = -0.25;    // Y 座標
    pose.orientation.w = 1.0;   // 姿態（四元數）
    object.pose = pose;

    // 將物件加入規劃場景
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

// 執行任務：初始化 -> 規劃 -> 執行
void MTCTaskNode::doTask() {
    task_ = createTask();  // 建立任務

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
    task.stages()->setName("demo task");  // 設定任務名稱
    task.loadRobotModel(node_);           // 載入機器人模型

    // 定義機械臂和夾爪的群組名稱
    const auto& arm_group_name = "panda_arm";   // 機械臂群組
    const auto& hand_group_name = "hand";       // 夾爪群組
    const auto& hand_frame = "panda_hand";      // 夾爪座標系

    // Set task properties
    // 設定任務屬性
    task.setProperty("group", arm_group_name);   // 主要控制的群組
    task.setProperty("eef", hand_group_name);    // 末端執行器（End Effector）
    task.setProperty("ik_frame", hand_frame);    // 逆運動學參考座標系

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
    cartesian_planner->setMaxVelocityScalingFactor(1.0);      // 最大速度比例
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);  // 最大加速度比例
    cartesian_planner->setStepSize(.01);                      // 步進大小 0.01m

    // 階段 2：打開夾爪
    // 使用關節插值規劃器將夾爪移動到 "open" 姿態
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>(
        "open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);  // 設定要控制的群組
    stage_open_hand->setGoal("open");            // 目標姿態名稱
    task.add(std::move(stage_open_hand));        // 將階段加入任務

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
            executor.spin();  // 開始處理 ROS2 訊息
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