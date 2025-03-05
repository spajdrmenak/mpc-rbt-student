#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.frame = "Hello World!";

  data.x = rand();
  data.y = rand();
  data.z = rand();

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };

  Utils::Message::serialize(frame, data);
  send(frame);

  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
  RCLCPP_INFO(logger, "\n\thodnota x: %f", data.x);
  RCLCPP_INFO(logger, "\n\thodnota y: %f", data.y);
  RCLCPP_INFO(logger, "\n\thodnota z: %f", data.z);
}
