#include <spdlog/spdlog.h>
#include "opencv2/core.hpp"

#include <sam2_segmentation/sam2_segmentation.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
        std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("picknik_ur_site_config");
        const std::filesystem::path encoder_onnx_file = package_path/"models"/"sam2_hiera_large_encoder.onnx";
        const std::filesystem::path decoder_onnx_file = package_path/"models"/"decoder.onnx";
        sam2_ = std::make_shared<moveit_pro_ml::SAM2>(encoder_onnx_file, decoder_onnx_file);
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

    void SAM2Segmentation::convert_image_to_onnx(const sensor_msgs::msg::Image& image_msg)
    {
        // onnx_image_
        onnx_image_.shape = {image_msg.height, image_msg.width, 3};
        onnx_image_.data.resize(image_msg.height*image_msg.width*3);
        for (size_t i = 0; i < onnx_image_.data.size();++i)
        {
            onnx_image_.data[i] = static_cast<float>(image_msg.data[i])/255.0f;
        }
        onnx_image_ = moveit_pro_ml::permute_image_data(onnx_image_);
    }

    BT::NodeStatus SAM2Segmentation::tick()
    {
        const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>(kPortImage),
            getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint));

        // Check that all required input data ports were set.
        if (!ports.has_value())
        {
            spdlog::error("Failed to get required values from input data ports:\n{}", ports.error());
            return BT::NodeStatus::FAILURE;
        }
        const auto& [image_msg, points_2d] = ports.value();

        if (image_msg.encoding!= "rgb8" && image_msg.encoding!= "rgba8")
        {
            spdlog::error("Invalid image message format. Expected (rgb8, rgba8) got :\n{}", image_msg.encoding);
            return BT::NodeStatus::FAILURE;
        }

        // create ONNX formatted image tensor from ROS image
        convert_image_to_onnx(image_msg);

        std::vector<moveit_pro_ml::PointPrompt> point_prompts;
        for (auto const& point : points_2d)
        {
            // assume all point are the same label
            point_prompts.push_back({{static_cast<float>(point.point.x), static_cast<float>(point.point.y)}, {1.0f}});
        }

        auto masks = sam2_->predict(onnx_image_, point_prompts);

        return BT::NodeStatus::SUCCESS;
    }

    BT::KeyValueVector SAM2Segmentation::metadata()
    {
        return {{"description", "TODO `"}};
    }
} // namespace sam2_segmentation
