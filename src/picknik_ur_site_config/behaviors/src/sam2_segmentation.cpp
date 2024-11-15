#include <spdlog/spdlog.h>
#include "opencv2/core.hpp"

#include <sam2_segmentation/sam2_segmentation.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <sensor_msgs/msg/image.hpp>
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

    BT::NodeStatus SAM2Segmentation::tick()
    {
        const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>(kPortImage),
            getInput<std::string>(kPortPoint));

        // Check that all required input data ports were set.
        if (!ports.has_value())
        {
            spdlog::error("Failed to get required values from input data ports:\n{}", ports.error());
            return BT::NodeStatus::FAILURE;
        }

        const auto& [image_msg, points_2d] = ports.value();
        cv::Mat image(image_msg.height, image_msg.width, CV_8UC3, (void*) image_msg.data.data());
        cv::Mat image_float;
        image.convertTo(image_float, CV_32FC3, (1.0/255.0));

        std::shared_ptr<moveit_pro_ml::ONNXImage> onnx_image;

        onnx_image->data = std::vector<float>((float*)image_float.data,
                                                      ((float*)image_float.data) + (image_float.rows * image_float.cols *
                                                          image_float.
                                                          channels()));
        onnx_image->shape = {image_float.rows, image_float.cols, image_float.channels()};
        onnx_image = permute_image_data(onnx_image);
        //
        //
        // std::vector<std::array<float, 2>> points;
        auto masks = sam2_->predict(onnx_image, points);

        return BT::NodeStatus::SUCCESS;
    }

    BT::KeyValueVector SAM2Segmentation::metadata()
    {
        return {{"description", "TODO `"}};
    }
} // namespace sam2_segmentation
