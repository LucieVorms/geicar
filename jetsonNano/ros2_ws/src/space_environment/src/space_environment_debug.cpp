// #include "space_environment_debug.hpp"
//
// #define SPACE_ENVIRONMENT_DEBUG
//
// SpaceEnvironmentDebug::SpaceEnvironmentDebug(const rclcpp::Logger& logger) : logger(logger) {}
//
// void SpaceEnvironmentDebug::log_new_scan() {
// #ifdef SPACE_ENVIRONMENT_DEBUG
//     RCLCPP_INFO(logger, "==== NEW SCAN ====");
// #endif
// }
//
// void SpaceEnvironmentDebug::log_init(const double rad, const double range) {
// #ifdef SPACE_ENVIRONMENT_DEBUG
//     RCLCPP_INFO(logger, "with rad: %f", rad);
//     RCLCPP_INFO(logger, "   range: %f", range);
// #endif
// }
//
// void SpaceEnvironmentDebug::log_space(const bool valid, const double value) {
// #ifdef SPACE_ENVIRONMENT_DEBUG
//     RCLCPP_INFO(logger, "          range %s %f", valid ? ">=" : "<", value);
// #endif
// }