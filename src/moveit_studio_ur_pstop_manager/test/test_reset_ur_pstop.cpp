// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

/**
 * Test to verify the reset protective stop functionality in the UR protective stop manager node.
 */

#include <chrono>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr auto kResetProtectiveStopService = "recover_from_protective_stop";
}  // namespace

namespace moveit_studio_ur_pstop_manager
{

using namespace std::chrono_literals;

/**
 * @brief Test class to setup service client to reset UR protective stop.
 */
class ResetURProtectiveStopTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_reset_ur_pstop");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Spin up an executor thread.
    executor_thread_ = std::thread([this]() {
      executor_->add_node(node_);
      executor_->spin();
      executor_->remove_node(node_);
    });
  }

  void TearDown() override
  {
    // Clean up execution.
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  bool setupResetProtectiveStopClient()
  {
    reset_ur_protective_stop_client_ = node_->create_client<std_srvs::srv::Trigger>(kResetProtectiveStopService);
    return reset_ur_protective_stop_client_->wait_for_service(5s);
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_ur_protective_stop_client_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread executor_thread_;
};

TEST_F(ResetURProtectiveStopTestFixture, ResetURProtectiveStop)
{
  // GIVEN client to reset UR protective stop
  ASSERT_TRUE(setupResetProtectiveStopClient());

  // WHEN a request to clear protective stop state is sent
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response_future = reset_ur_protective_stop_client_->async_send_request(request);

  // THEN a success response is received indicating the protective stop is cleared.
  ASSERT_EQ(response_future.wait_for(10s), std::future_status::ready);
  auto response = response_future.get();
  ASSERT_TRUE(response->success);
}

}  // namespace moveit_studio_ur_pstop_manager
