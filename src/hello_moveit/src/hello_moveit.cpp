// 引入 MoveIt 視覺化工具標頭檔
#include <moveit_visual_tools/moveit_visual_tools.h>

// 引入標準函式庫
#include <memory>                                               // 智慧指標
#include <moveit/move_group_interface/move_group_interface.hpp> // MoveIt 動作群組介面
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp> // ROS2 C++ 函式庫
#include <thread>            // 多線程支援

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    // 初始化 ROS2 系統
    rclcpp::init(argc, argv);

    // 建立一個名為 "hello_moveit" 的 ROS2 節點
    // 並自動從參數檔案中讀取參數設定
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));

    // Create a ROS logger
    // 建立日誌記錄器，用於輸出訊息
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with
    // ROS 建立單線程執行器，用於處理 ROS2 的訊息與服務
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); // 將節點加入執行器
    // 在獨立線程中啟動執行器，讓它持續運行
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    // 建立 MoveIt 的動作群組介面，用於控制名為 "manipulator" 的機械臂
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "manipulator");

    // Construct and initialize MoveItVisualTools
    // 建立並初始化 MoveIt 視覺化工具
    // 用於在 RViz 中顯示機械臂的狀態、軌跡等資訊
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers(); // 清除 RViz 中所有舊的標記
    moveit_visual_tools
        .loadRemoteControl(); // 載入遠端控制面板（RViz 中的 Next 按鈕）

    // Create closures for visualization
    // 定義一個函數：在 RViz 中顯示標題文字
    auto const draw_title = [&moveit_visual_tools](auto text) {
        // 設定文字的顯示位置
        auto const text_pose = [] {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0; // Place text 1m above the base link /
                                         // 將文字放在基座上方 1 公尺處
            return msg;
        }();
        // 發布文字到 RViz（白色、超大字體）
        moveit_visual_tools.publishText(text_pose, text,
                                        rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
    };

    // 定義一個函數：顯示提示訊息並等待使用者按下 "Next" 按鈕
    auto const prompt = [&moveit_visual_tools](auto text) {
        moveit_visual_tools.prompt(text);
    };

    // 定義一個函數：在 RViz 中繪製機械臂的運動軌跡
    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group_interface.getRobotModel()->getJointModelGroup(
             "manipulator")](auto const trajectory) {
            moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

    // Set a target Pose
    // 設定目標位姿（Pose）
    auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.y = 0.8;
        msg.orientation.w = 0.6;
        msg.position.x = 0.1;
        msg.position.y = 0.4;
        msg.position.z = 0.4;
        return msg;
    }();
    move_group_interface.setPoseTarget(
        target_pose); // 告訴 MoveIt 要移動到這個目標位置

    // Create a plan to that target pose
    // 開始規劃移動路徑
    prompt(
        "Press 'Next' in the RvizVisualToolsGui window to plan"); // 等待使用者確認開始規劃
    draw_title("Planning");        // 顯示 "Planning" 標題
    moveit_visual_tools.trigger(); // 觸發視覺化更新

    // Create collision object for the robot to avoid
    auto const collision_object =
        [frame_id = move_group_interface.getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = frame_id;
            collision_object.id = "box1";
            shape_msgs::msg::SolidPrimitive primitive;

            // Define the size of the box in meters
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 0.5;
            primitive.dimensions[primitive.BOX_Y] = 0.1;
            primitive.dimensions[primitive.BOX_Z] = 0.5;

            // Define the pose of the box (relative to the frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w =
                1.0; // We can leave out the x, y, and z components of the
                     // quaternion since they are initialized to 0
            box_pose.position.x = 0.2;
            box_pose.position.y = 0.2;
            box_pose.position.z = 0.25;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            return collision_object;
        }();
    // Add the collision object to the scene 把碰撞加入到場景中
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
    // 呼叫 MoveIt 進行路徑規劃
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok =
            static_cast<bool>(move_group_interface.plan(msg)); // 執行規劃
        return std::make_pair(ok, msg); // 回傳規劃是否成功以及規劃結果
    }();

    // Execute the plan
    // 根據規劃結果決定下一步動作
    if (success) { // 如果規劃成功
        draw_trajectory_tool_path(
            plan.trajectory); // 在 RViz 中繪製規劃好的軌跡
        moveit_visual_tools.trigger();
        prompt(
            "Press 'Next' in the RvizVisualToolsGui window to execute"); // 等待使用者確認執行
        draw_title("Executing"); // 顯示 "Executing" 標題
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan); // 執行規劃好的動作！
    } else {                                // 如果規劃失敗
        draw_title("Planning Failed!");     // 顯示失敗訊息
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!"); // 在終端機輸出錯誤訊息
    }

    // Shutdown ROS
    // 關閉 ROS2 系統
    rclcpp::shutdown();
    spinner.join(); // 等待執行器線程結束
    return 0;       // 程式正常結束
}