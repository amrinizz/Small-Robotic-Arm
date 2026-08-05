#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(const std::string& task_name, const std::string& object_id, float targetXpos, float targetYpos);

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(const std::string& task_name, const std::string& object_id, float targetXpos, float targetYpos);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}
void MTCTaskNode::setupPlanningScene()
{ 
  moveit_msgs::msg::CollisionObject object1, object2, object3, object4, object5; 
  object1.id = "object1";
  object1.header.frame_id = "world";
  object1.primitives.resize(1);
  object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object1.primitives[0].dimensions = { 0.05, 0.05, 0.05 };

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.25;
  pose1.position.y = -0.35;
  pose1.orientation.w = 1.0;
  object1.pose = pose1;

  object2.id = "object2";
  object2.header.frame_id = "world";
  object2.primitives.resize(1);
  object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object2.primitives[0].dimensions = { 0.05, 0.05, 0.05 };

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.25;
  pose2.position.y = -0.2;
  pose2.orientation.w = 1.0;
  object2.pose = pose2;

  object3.id = "object3";
  object3.header.frame_id = "world";
  object3.primitives.resize(1);
  object3.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object3.primitives[0].dimensions = { 0.05, 0.05, 0.05 };

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.25;
  pose3.position.y = 0.1;
  pose3.orientation.w = 1.0;
  object3.pose = pose3;  

  object4.id = "object4";
  object4.header.frame_id = "world";
  object4.primitives.resize(1);
  object4.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object4.primitives[0].dimensions = { 0.05, 0.05, 0.05 };

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.25;
  pose4.position.y = 0.25;
  pose4.orientation.w = 1.0;
  object4.pose = pose4;

  object5.id = "object5";
  object5.header.frame_id = "world";
  object5.primitives.resize(1);
  object5.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object5.primitives[0].dimensions = { 0.02, 1.0, 0.15 }; // wlh

  geometry_msgs::msg::Pose pose5;
  pose5.position.x = 0.45;
  pose5.position.y = -0.12;
  pose5.position.z = 0.3;
  pose5.orientation.w = 1.0;
  object5.pose = pose5;

  // Create a PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface psi;

  // Apply the collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {object1, object2, object3, object4, object5};
  psi.applyCollisionObjects(collision_objects);
}

void MTCTaskNode::doTask(const std::string& task_name, const std::string& object_id, float targetXpos, float targetYpos)
{
  task_ = createTask(task_name, object_id, targetXpos,  targetYpos);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask(const std::string& task_name,const std::string& object_id, float targetXpos, float targetYpos)
{
  mtc::Task task;
  task.stages()->setName(task_name);
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_id);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object_id,
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_id, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_id);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = object_id;
      target_pose_msg.pose.position.x = targetXpos;
      target_pose_msg.pose.position.y = targetYpos;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object_id);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object_id,
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_id, hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });
  
  mtc_task_node->setupPlanningScene();
  //Incremental movement  
  mtc_task_node->doTask("Task 1","object1", 0.25, 0.0);
  mtc_task_node->doTask("Task 2","object2", 0.25, 0.0);
  mtc_task_node->doTask("Task 3","object3", 0.25, 0.0);
  mtc_task_node->doTask("Task 4","object4", 0.25, 0.0);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
