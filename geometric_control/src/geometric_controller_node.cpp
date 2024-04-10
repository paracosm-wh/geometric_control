#include "geometric_control/geometric_controller.h"

int main(int argc, char** argv) {

    // ADD
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<geometricCtrl>());
    rclcpp::shutdown();
  return 0;
}
