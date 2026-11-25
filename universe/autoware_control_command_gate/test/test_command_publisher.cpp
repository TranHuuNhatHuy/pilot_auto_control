// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "command/publisher.hpp"
#include "test_fixtures.hpp"

#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::control_command_gate::test
{

namespace
{

namespace control_spec = autoware::component_interface_specs::control;

struct ControlSubscriptionProbe
{
  std::optional<Control> received;
  std::atomic<int> incompatible_qos_count{0};
  rclcpp::Subscription<Control>::SharedPtr subscription;
};

class CommandPublisherFixture
{
public:
  explicit CommandPublisherFixture(const std::string & name_prefix)
  : topic_name_("/" + name_prefix + "_pub/output/control"),
    pub_node_(std::make_shared<autoware::agnocast_wrapper::Node>(name_prefix + "_pub")),
    sub_node_(std::make_shared<rclcpp::Node>(name_prefix + "_sub")),
    publisher_(*pub_node_)
  {
    publisher_.set_prev_control(std::make_shared<Control>());
    if (
      publisher_.create_diag_task(TimeoutDiag::Params{0.5, 1.0}, *pub_node_->get_clock()) ==
      nullptr) {
      throw std::runtime_error("failed to create timeout diag task");
    }
    executor_.add_node(pub_node_->get_node_base_interface());
    executor_.add_node(sub_node_);
  }

  ~CommandPublisherFixture()
  {
    executor_.remove_node(sub_node_);
    executor_.remove_node(pub_node_->get_node_base_interface());
  }

  std::unique_ptr<ControlSubscriptionProbe> create_control_probe(const rclcpp::QoS & qos)
  {
    auto probe = std::make_unique<ControlSubscriptionProbe>();
    rclcpp::SubscriptionOptions opts;
    opts.event_callbacks.incompatible_qos_callback =
      [raw_probe = probe.get()](const rclcpp::QOSRequestedIncompatibleQoSInfo &) {
        ++raw_probe->incompatible_qos_count;
      };
    probe->subscription = sub_node_->create_subscription<Control>(
      topic_name_, qos,
      [raw_probe = probe.get()](const Control::ConstSharedPtr msg) { raw_probe->received = *msg; },
      opts);
    return probe;
  }

  CommandPublisher & publisher() { return publisher_; }
  rclcpp::executors::SingleThreadedExecutor & executor() { return executor_; }

private:
  std::string topic_name_;
  std::shared_ptr<autoware::agnocast_wrapper::Node> pub_node_;
  std::shared_ptr<rclcpp::Node> sub_node_;
  CommandPublisher publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

Control make_sample_control()
{
  Control cmd;
  cmd.longitudinal.velocity = 5.5f;
  cmd.longitudinal.acceleration = 0.8f;
  cmd.lateral.steering_tire_angle = -0.15f;
  return cmd;
}

// Confirms pubs created by CommandPublisher have same QoS as specified in component interface specs.
TEST(CommandPublisherQosTest, OutputPublishersMatchComponentInterfaceSpecs)
{
  // Arrange
  const auto node =
    std::make_shared<autoware::agnocast_wrapper::Node>("test_command_publisher_qos");
  const auto rclcpp_node = node->get_rclcpp_node();
  struct ExpectedEndpointQos
  {
    std::string topic_name;
    rclcpp::ReliabilityPolicy reliability;
    rclcpp::DurabilityPolicy durability;
  };
  const std::vector<ExpectedEndpointQos> expected_endpoints = {
    {"/test_command_publisher_qos/output/control", rclcpp::ReliabilityPolicy::Reliable,
     rclcpp::DurabilityPolicy::TransientLocal},
    {"/test_command_publisher_qos/output/gear", rclcpp::ReliabilityPolicy::Reliable,
     rclcpp::DurabilityPolicy::Volatile},
    {"/test_command_publisher_qos/output/turn_indicators", rclcpp::ReliabilityPolicy::Reliable,
     rclcpp::DurabilityPolicy::Volatile},
    {"/test_command_publisher_qos/output/hazard_lights", rclcpp::ReliabilityPolicy::Reliable,
     rclcpp::DurabilityPolicy::Volatile},
  };

  // Act
  CommandPublisher publisher(*node);
  std::vector<std::vector<rclcpp::TopicEndpointInfo>> actual_endpoints;
  actual_endpoints.reserve(expected_endpoints.size());
  for (const auto & expected : expected_endpoints) {
    actual_endpoints.push_back(rclcpp_node->get_publishers_info_by_topic(expected.topic_name));
  }

  // Assert
  for (size_t i = 0; i < expected_endpoints.size(); ++i) {
    const auto & expected = expected_endpoints[i];
    const auto & endpoints = actual_endpoints[i];
    ASSERT_EQ(endpoints.size(), 1u) << "topic=" << expected.topic_name;
    const auto & qos = endpoints.front().qos_profile();
    EXPECT_EQ(qos.reliability(), expected.reliability) << "topic=" << expected.topic_name;
    EXPECT_EQ(qos.durability(), expected.durability) << "topic=" << expected.topic_name;
  }
}

// Confirms a control command published with a VOLATIVE QoS is delivered to a sub
// that subs after command is published.
TEST(CommandPublisherQosTest, DeliversControlCommandToVolatileSubscriber)
{
  // Arrange
  CommandPublisherFixture fixture("test_command_publisher_volatile");
  const auto probe = fixture.create_control_probe(rclcpp::QoS(1).reliable().durability_volatile());
  ASSERT_TRUE(spin_until(
    fixture.executor(), [&probe]() { return probe->subscription->get_publisher_count() == 1u; }));
  const Control expected = make_sample_control();

  // Act
  fixture.publisher().on_control(main_id, expected);
  const bool delivered =
    spin_until(fixture.executor(), [&probe]() { return probe->received.has_value(); });

  // Assert
  ASSERT_TRUE(delivered);
  EXPECT_FLOAT_EQ(probe->received->longitudinal.velocity, expected.longitudinal.velocity);
  EXPECT_FLOAT_EQ(probe->received->longitudinal.acceleration, expected.longitudinal.acceleration);
  EXPECT_FLOAT_EQ(
    probe->received->lateral.steering_tire_angle, expected.lateral.steering_tire_angle);
  EXPECT_EQ(probe->incompatible_qos_count.load(), 0);
}

// Confirms a control command published with a TRANSIENT_LOCAL QoS is delivered to a sub
// that subs after command is published.
TEST(CommandPublisherQosTest, DeliversLatchedControlCommandToLateJoiningTransientLocalSubscriber)
{
  // Arrange
  CommandPublisherFixture fixture("test_command_publisher_transient_local");
  const Control expected = make_sample_control();
  fixture.publisher().on_control(main_id, expected);

  // Act: subscribe after publish with the same QoS as adapi.node.vehicle_command.
  const auto probe = fixture.create_control_probe(
    autoware::component_interface_specs::get_qos<control_spec::ControlCommand>());
  const bool delivered =
    spin_until(fixture.executor(), [&probe]() { return probe->received.has_value(); });

  // Assert
  ASSERT_TRUE(delivered);
  EXPECT_FLOAT_EQ(probe->received->longitudinal.velocity, expected.longitudinal.velocity);
  EXPECT_FLOAT_EQ(probe->received->longitudinal.acceleration, expected.longitudinal.acceleration);
  EXPECT_FLOAT_EQ(
    probe->received->lateral.steering_tire_angle, expected.lateral.steering_tire_angle);
  EXPECT_EQ(probe->incompatible_qos_count.load(), 0);
}

}  // namespace

}  // namespace autoware::control_command_gate::test
