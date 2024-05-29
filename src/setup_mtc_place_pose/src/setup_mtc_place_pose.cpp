#include <setup_mtc_place_pose/setup_mtc_place_pose.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/moveit_tools.hpp>
#include <moveit_studio_common/utils/yaml_parsing_tools.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene/planning_scene.h>

namespace setup_mtc_place_pose
{
    inline constexpr auto kDescriptionSetupMTCPlacePose = R"(
                <p>
                    Given an existing MTC Task object and an attached target object, appends MTC stages to place the object.
                </p>
            )";
    using GraspableObject = moveit_studio_vision_msgs::msg::GraspableObject;
    using namespace moveit::task_constructor;

    const auto kLogger = rclcpp::get_logger("SetupMTCPlacePose");

    // Port names for input and output ports.
    constexpr auto kPortIDTask = "task";
    constexpr auto kPortIDTargetObject = "target_object";
    constexpr auto kPortIDPlacePose = "place_pose";

    SetupMtcPlacePose::SetupMtcPlacePose(
            const std::string& name, const BT::NodeConfiguration& config,
            const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
        : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
    {
    }


    BT::PortsList SetupMtcPlacePose::providedPorts()
    {
        return { BT::InputPort<GraspableObject>(kPortIDTargetObject, "{object}",
                "Cuboid object to place, represented as a represented as a "
                "moveit_studio_vision_msgs/GraspableObject."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPlacePose, "{place_pose}", "Object place pose."),
        BT::BidirectionalPort<TaskPtr>(kPortIDTask, "{mtc_task}",
                "MoveIt Task Constructor task.") };
    }

    BT::KeyValueVector SetupMtcPlacePose::metadata()
    {
        return { { "subcategory", "Task Planning" }, { "description", kDescriptionSetupMTCPlacePose } };
    }

    BT::NodeStatus SetupMtcPlacePose::tick()
    {

        // ----------------------------------------
        // Load data from the behavior input ports.
        // ----------------------------------------
        const auto target_object = getInput<GraspableObject>(kPortIDTargetObject);
        const auto target_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPlacePose);
        const auto task = getInput<TaskPtr>(kPortIDTask);


        // Check that all required input data ports were set
        if (const auto error = moveit_studio::behaviors::maybe_error(target_object, task))
        {
            shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                    error.value());
            return BT::NodeStatus::FAILURE;
        }

        // Inputs
        // NOTE assumes box object for now! TODO check for validity
        // TODO: make these parameters as well
        const double object_height = target_object.value().bounding_volume.dimensions[2];
        const std::string object_name = target_object.value().id;
        const double place_surface_offset = 0.005;
        const std::string hand_frame = "grasp_link";
        const std::string world_frame = "world";
        const std::string group_name = "manipulator";
        const std::string hand_group_name = "gripper";
        const std::string end_effector_name = "moveit_ee";
        const std::string hand_open_pose = "open";

        // Cartesian planner
        auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.01);

        const auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(
                shared_resources_->node, "ompl", "RRTConnectkConfigDefault");



        /******************************************************
          ---- *          no-op motion for monitoring                              *
         *****************************************************/
        Stage* before_place_ptr = nullptr;
        {
            auto stage = std::make_unique<stages::MoveRelative>("no-op", cartesian_planner);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->setProperty("group", group_name);
            stage->setMinMaxDistance(.0, .005);

            // Set downward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = world_frame;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            before_place_ptr = stage.get();
            task.value()->insert(std::move(stage));
        }


        /******************************************************
         *                                                    *
         *          Move to Place                             *
         *                                                    *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::Connect>(
                    "move to place", stages::Connect::GroupPlannerVector{ { group_name, sampling_planner } });
            stage->setTimeout(5.0);
            stage->properties().configureInitFrom(Stage::PARENT);
            task.value()->add(std::move(stage));
        }

        {
            /******************************************************
             *                                                    *
             *          Place Object                              *
             *                                                    *
             *****************************************************/
            {
                auto place = std::make_unique<SerialContainer>("place object");
                place->properties().configureInitFrom(Stage::PARENT);
                place->setProperty("hand", hand_group_name);
                place->setProperty("eef", end_effector_name);
                place->setProperty("group", group_name);

                /******************************************************
                  ---- *          Lower Object                              *
                 *****************************************************/
                {
                    auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
                    stage->properties().set("marker_ns", "lower_object");
                    stage->properties().set("link", hand_frame);
                    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(.03, .13);

                    // Set downward direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = world_frame;
                    vec.vector.z = -1.0;
                    stage->setDirection(vec);
                    place->insert(std::move(stage));
                }

                /******************************************************
                  ---- *          Generate Place Pose                       *
                 *****************************************************/
                {
                    // Generate Place Pose
                    auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
                    stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
                    stage->properties().set("marker_ns", "place_pose");
                    stage->setObject(object_name);

                    // Set target pose
                    geometry_msgs::msg::PoseStamped p = target_pose.value();
                    p.pose.position.z += 0.5 * object_height + place_surface_offset;
                    stage->setPose(p);

                    // Hook into the planning scene state of the first stage added by this Behavior.
                    stage->setMonitoredStage(before_place_ptr);

                    // Compute IK
                    auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
                    wrapper->setMaxIKSolutions(2);
                    wrapper->setIKFrame(object_name);
                    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
                    wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
                    place->insert(std::move(wrapper));
                }

                /******************************************************
                  ---- *          Open Hand                              *
                 *****************************************************/
                {
                    auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
                    stage->setGroup(hand_group_name);
                    stage->setGoal(hand_open_pose);
                    place->insert(std::move(stage));
                }

                /******************************************************
                  ---- *          Forbid collision (hand, object)        *
                 *****************************************************/
                {
                    auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
                    stage->allowCollisions(object_name, *task.value()->getRobotModel()->getJointModelGroup(hand_group_name), false);
                    place->insert(std::move(stage));
                }

                /******************************************************
                  ---- *          Detach Object                             *
                 *****************************************************/
                {
                    auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
                    stage->detachObject(object_name, hand_frame);
                    place->insert(std::move(stage));
                }

                /******************************************************
                  ---- *          Retreat Motion                            *
                 *****************************************************/
                {
                    auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
                    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(.12, .25);
                    stage->setIKFrame(hand_frame);
                    stage->properties().set("marker_ns", "retreat");
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = hand_frame;
                    vec.vector.z = -1.0;
                    stage->setDirection(vec);
                    place->insert(std::move(stage));
                }

                // Add place container to task
                task.value()->add(std::move(place));
            }

        }

        return BT::NodeStatus::SUCCESS;
    }

}  // namespace setup_mtc_place_pose
