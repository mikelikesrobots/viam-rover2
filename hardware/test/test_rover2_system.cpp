#include <gmock/gmock.h>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestRover2System : public ::testing::Test {
 protected:
  void SetUp() override {
    mock_system_ =
        R"(
  <ros2_control name="MockRover2SystemHardware" type="system">
    <hardware>
      <plugin>viam_rover2/Rover2SystemHardware</plugin>
      <param name="use_mock_hardware">true</param>
      <param name="serial_port">/dev/ttyS0</param>
      <param name="baud_rate">115200</param>
    </hardware>
  </ros2_control>
)";
    clock_ = std::make_shared<rclcpp::Clock>();
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TestRover2System"));
  }

  std::string mock_system_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
};

TEST_F(TestRover2System, load_generic_system) {
  auto urdf =
      ros2_control_test_assets::urdf_head + mock_system_ + ros2_control_test_assets::urdf_tail;

  std::unique_ptr<hardware_interface::ResourceManager> rm;
  ASSERT_NO_THROW(
      rm = std::make_unique<hardware_interface::ResourceManager>(urdf, clock_, *logger_));
  ASSERT_TRUE(rm);
  EXPECT_GT(rm->system_components_size(), 0u);
}
