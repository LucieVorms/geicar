#ifndef SPACE_ENVIRONMENT_DEBUG_HPP
#define SPACE_ENVIRONMENT_DEBUG_HPP

#include <rclcpp/logging.hpp>

class SpaceEnvironmentDebug {
public:
    explicit SpaceEnvironmentDebug(const rclcpp::Logger& logger);

    void log_new_scan();

    void log_init(double rad, double range);

    void log_space(bool valid, double value);

private:
    rclcpp::Logger logger;
};

#endif // SPACE_ENVIRONMENT_DEBUG_HPP
