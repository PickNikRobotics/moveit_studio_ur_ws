// Copyright (c) 2023 PickNik Inc.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
