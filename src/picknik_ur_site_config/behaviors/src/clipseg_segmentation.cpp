#include <spdlog/spdlog.h>
#include "opencv2/core.hpp"

#include <clipseg_segmentation/clipseg_segmentation.hpp>
#include <moveit_pro_ml/onnx_utils.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>

namespace
{
  constexpr auto kPortImage = "image";
  constexpr auto kPortImageDefault = "{image}";
  constexpr auto kPortPrompt = "prompt";
  constexpr auto kPortPromptDefault = "{prompt}";
  constexpr auto kPortMasks = "masks2d";
  constexpr auto kPortMasksDefault = "{masks2d}";
} // namespace

namespace custom_behaviors
{
  ClipSegSegmentation::ClipSegSegmentation(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
    : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
  {
    std::filesystem::path package_path = ament_index_cpp::get_package_share_directory("picknik_ur_site_config");
    const std::filesystem::path clip_model_onnx_file = package_path / "models" / "clip.onnx";
    const std::filesystem::path clipseg_model_onnx_file = package_path / "models" / "clipseg.onnx";
    clipseg_ = std::make_shared<moveit_pro_ml::ClipSeg>(clip_model_onnx_file, clipseg_model_onnx_file);
  }

  BT::PortsList ClipSegSegmentation::providedPorts()
  {
    return {
      BT::InputPort<sensor_msgs::msg::Image>(kPortImage, kPortImageDefault,
                                             "The Image to run segmentation on."),
      BT::InputPort<std::vector<std::string>>(kPortPrompt, kPortPromptDefault,
                                                                   "The input prompt as a <code>std::string</code> to be used for segmentation."),

  BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortMasks, kPortMasksDefault,
                                                                   "The masks contained in a vector of <code>moveit_studio_vision_msgs::msg::Mask2D</code> messages.")


    };
  }

  void ClipSegSegmentation::set_onnx_from_ros_image(const sensor_msgs::msg::Image& image_msg)
  {
    onnx_image_.shape = {image_msg.height, image_msg.width, 3};
    onnx_image_.data.resize(image_msg.height * image_msg.width * 3);
    for (size_t i = 0; i < onnx_image_.data.size(); ++i)
    {
      onnx_image_.data[i] = static_cast<float>(image_msg.data[i]) / 255.0f;
    }
  }

  void ClipSegSegmentation::set_ros_image_from_onnx(const moveit_pro_ml::ONNXImage& onnx_image)
  {
    image_.height = static_cast<uint32_t>(onnx_image.shape[0]);
    image_.width = static_cast<uint32_t>(onnx_image.shape[1]);
    image_.encoding = "mono8";
    image_.data.resize(image_.height * image_.width);
    image_.step = image_.width;
    for (size_t i = 0; i < image_.data.size(); ++i)
    {
      image_.data[i] = onnx_image.data[i]>0.5 ? 255: 0;
    }

    cv::Mat masks_scaled(image_.height, image_.width, CV_8UC1, image_.data.data());
    cv::imwrite("/tmp/mask.png", masks_scaled); // TODO delete this

   }


  tl::expected<bool, std::string> ClipSegSegmentation::doWork()
  {
    const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>(kPortImage),
                                                                   getInput<std::vector<
                                                                     std::string>>(kPortPrompt));

    // Check that all required input data ports were set.
    if (!ports.has_value())
    {
      auto error_message = fmt::format("Failed to get required values from input data ports:\n{}", ports.error());
      return tl::make_unexpected(error_message);
    }
    const auto& [image_msg, prompts] = ports.value();

    if (image_msg.encoding != "rgb8")
    {
      auto error_message = fmt::format("Invalid image message format. Expected `rgb8` got :\n{}", image_msg.encoding);
      return tl::make_unexpected(error_message);
    }

    // create ONNX formatted image tensor from ROS image
    set_onnx_from_ros_image(image_msg);
    try
    {
      onnx_image_.shape = {1, onnx_image_.shape[0], onnx_image_.shape[1], onnx_image_.shape[2]};
      auto logits = clipseg_->predict(onnx_image_, prompts);
      logits.shape = {logits.shape[1], logits.shape[2]};
      logits = moveit_pro_ml::resize_image(logits, {image_msg.height, image_msg.width});

      moveit_studio_vision_msgs::msg::Mask2D our_mask;
      image_.header = image_msg.header;
      set_ros_image_from_onnx(logits);
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

  BT::KeyValueVector ClipSegSegmentation::metadata()
  {
    return {
      {
        "description",
        "Segments a ROS image message using the text prompt provided as a <code>std::string</code>."
      }
    };
  }
} // namespace sam2_segmentation
