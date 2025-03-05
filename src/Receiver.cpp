#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  Utils::Message::deserialize(frame, data);

  RCLCPP_INFO(logger, "Recieveing data from host: '%s:%d'", frame.address.c_str(), frame.port);
  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
  RCLCPP_INFO(logger, "\n\tzprava: %s", data.frame.c_str());
  RCLCPP_INFO(logger, "\n\thodnota x: %f", data.x);
  RCLCPP_INFO(logger, "\n\thodnota y: %f", data.y);
  RCLCPP_INFO(logger, "\n\thodnota z: %f", data.z);
}
