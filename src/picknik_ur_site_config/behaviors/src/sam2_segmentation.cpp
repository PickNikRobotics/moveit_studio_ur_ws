#include <spdlog/spdlog.h>
#include "opencv2/core.hpp"

#include <sam2_segmentation/sam2_segmentation.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>

namespace
{
  constexpr auto kPortImage = "image";
  constexpr auto kPortImageDefault = "{image}";
  constexpr auto kPortPoint = "pixel_coords";
  constexpr auto kPortPointDefault = "{pixel_coords}";
  constexpr auto kPortMasks = "masks2d";
  constexpr auto kPortMasksDefault = "{masks2d}";
} // namespace

namespace custom_behaviors
{
  SAM2Segmentation::SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
    : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
  {
    std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("picknik_ur_site_config");
    const std::filesystem::path encoder_onnx_file = package_path / "models" / "sam2_hiera_large_encoder.onnx";
    const std::filesystem::path decoder_onnx_file = package_path / "models" / "decoder.onnx";
    sam2_ = std::make_shared<moveit_pro_ml::SAM2>(encoder_onnx_file, decoder_onnx_file);
  }

  BT::PortsList SAM2Segmentation::providedPorts()
  {
    return {
      BT::InputPort<sensor_msgs::msg::Image>(kPortImage, kPortImageDefault,
                                             "The Image to run segmentation on."),
      BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortPoint, kPortPointDefault,
                                                                   "The input points, as a vector of <code>geometry_msgs/PointStamped</code> messages to be used for segmentation."),

  BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortMasks, kPortMasksDefault,
                                                                   "The masks contained in a vector of <code>moveit_studio_vision_msgs::msg::Mask2D</code> messages.")


    };
  }

  void SAM2Segmentation::set_onnx_from_ros_image(const sensor_msgs::msg::Image& image_msg)
  {
    onnx_image_.shape = {image_msg.height, image_msg.width, 3};
    onnx_image_.data.resize(image_msg.height * image_msg.width * 3);
    for (size_t i = 0; i < onnx_image_.data.size(); ++i)
    {
      onnx_image_.data[i] = static_cast<float>(image_msg.data[i]) / 255.0f;
    }
    onnx_image_ = moveit_pro_ml::resize_image(onnx_image_, {1024, 1024, 3});
    onnx_image_ = moveit_pro_ml::permute_image_data(onnx_image_);
  }


  void SAM2Segmentation::set_ros_image_from_onnx(const moveit_pro_ml::ONNXImage& onnx_image)
  {
    image_.height = static_cast<uint32_t>(onnx_image.shape[0]);
    image_.width = static_cast<uint32_t>(onnx_image.shape[1]);
    image_.encoding = "mono8";
    image_.data.resize(image_.height * image_.width);
    image_.step = image_.width;
    for (size_t i = 0; i < image_.data.size(); ++i)
    {
      image_.data[i] = static_cast<uint8_t>((onnx_image.data[i]>0.5)*255);
    }

    cv::Mat masks_scaled(image_.height, image_.width, CV_8UC1, image_.data.data());
    // transpose(masks_scaled, masks_scaled);
    cv::imwrite("/tmp/mask.png", masks_scaled);


   }


  tl::expected<bool, std::string> SAM2Segmentation::doWork()
  {
    const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>(kPortImage),
                                                                   getInput<std::vector<
                                                                     geometry_msgs::msg::PointStamped>>(kPortPoint));

    // Check that all required input data ports were set.
    if (!ports.has_value())
    {
      auto error_message = fmt::format("Failed to get required values from input data ports:\n{}", ports.error());
      return tl::make_unexpected(error_message);
    }
    const auto& [image_msg, points_2d] = ports.value();

    if (image_msg.encoding != "rgb8" && image_msg.encoding != "rgba8")
    {
      auto error_message = fmt::format("Invalid image message format. Expected (rgb8, rgba8) got :\n{}", image_msg.encoding);
      return tl::make_unexpected(error_message);
    }

    // create ONNX formatted image tensor from ROS image
    set_onnx_from_ros_image(image_msg);

    std::vector<moveit_pro_ml::PointPrompt> point_prompts;
    for (auto const& point : points_2d)
    {
      // assume all point are the same label
      point_prompts.push_back({{1024*static_cast<float>(point.point.x), 1024*static_cast<float>(point.point.y)}, {1.0f}});
    }

    if (image_msg.encoding != "rgb8")
    {
      auto error_message = fmt::format("Invalid image message format. Expected `rgb8` got :\n{}", image_msg.encoding);
      return tl::make_unexpected(error_message);
    }
    try
    {
      auto masks = sam2_->predict(onnx_image_, point_prompts);
      masks = moveit_pro_ml::resize_image(masks, {image_msg.height, image_msg.width});

      moveit_studio_vision_msgs::msg::Mask2D our_mask;
      image_.header = image_msg.header;
      set_ros_image_from_onnx(masks);
      our_mask.pixels = image_;
      our_mask.x = 0;
      our_mask.y = 0;
      setOutput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortMasks, {our_mask});
    }
    catch (std::invalid_argument& e)
    {
      auto error_message = fmt::format("Invalid argument: {}", e.what());
      return tl::make_unexpected(error_message);
    }

    return true;
  }

  BT::KeyValueVector SAM2Segmentation::metadata()
  {
    return {
      {
        "description",
        "Segments a ROS image message using the click points provided as a vector of <code>geometry_msgs/PointStamped</code> messages."
      }
    };
  }
} // namespace sam2_segmentation
