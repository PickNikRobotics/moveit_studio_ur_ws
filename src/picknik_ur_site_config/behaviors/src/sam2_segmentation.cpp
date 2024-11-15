#include <geometry_msgs/msg/point_stamped.hpp>
#include <sam2_segmentation/sam2_segmentation.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace
{
    constexpr auto kPortImage = "image";
    constexpr auto kPortImageDefault = "{image}";
    constexpr auto kPortPoint = "pixel_coords";
    constexpr auto kPortPointDefault = "{pixel_coords}";
}// namespace

namespace sam2_segmentation
{
    SAM2Segmentation::SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
    {
    }

    BT::PortsList SAM2Segmentation::providedPorts()
    {
        return {
            BT::InputPort<sensor_msgs::msg::Image>(kPortImage, kPortImageDefault,
                                       "The Image to run segmentation on."),
            BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint, kPortPointDefault,
                           "The input points, as a vector of <code>geometry_msgs/PointStamped</code> messages to be used for segmentation.")
        };
    }

    BT::KeyValueVector SAM2Segmentation::metadata()
    {
        return {{"description", "TODO `"}};
    }
} // namespace sam2_segmentation
