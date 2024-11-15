#pragma once


#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_ml/onnx_sam2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fmt/format.h>

namespace custom_behaviors
{
/**
 * @brief Segment an image using the SAM 2 model
 */
class SAM2Segmentation : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
/**
   * @brief Constructor for the sam2_segmentation behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
 SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

 /**
  * @brief Implementation of the required providedPorts() function for the sam2_segmentation Behavior.
  * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
  * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
  * @return sam2_segmentation does not use expose any ports, so this function returns an empty list.
  */
 static BT::PortsList providedPorts();

 /**
  * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
  * subcategory, in the MoveIt Studio Developer Tool.
  * @return A BT::KeyValueVector containing the Behavior metadata.
  */
 static BT::KeyValueVector metadata();

protected:
  tl::expected<bool, std::string> doWork() override;


private:
 void set_onnx_from_ros_image(const sensor_msgs::msg::Image& image_msg);
 void set_ros_image_from_onnx(const moveit_pro_ml::ONNXImage& onnx_image);

 std::shared_ptr<moveit_pro_ml::SAM2> sam2_;
 moveit_pro_ml::ONNXImage onnx_image_;
 sensor_msgs::msg::Image image_;
 
  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;

};
}  // namespace sam2_segmentation
