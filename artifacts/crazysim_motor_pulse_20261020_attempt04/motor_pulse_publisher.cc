#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>

using Clock = std::chrono::steady_clock;
std::int64_t monotonicNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch()).count();
}
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const int selected = std::stoi(argv[1]);
  if (selected < 0 || selected > 3) return 2;
  std::ofstream log(argv[2]);
  if (!log) return 2;
  gz::transport::Node node;
  std::atomic<std::int64_t> simNs{-1};
  if (!node.Subscribe<gz::msgs::WorldStatistics>(
          "/world/crazysim_default/stats",
          [&simNs](const gz::msgs::WorldStatistics &s) {
            simNs.store(static_cast<std::int64_t>(s.sim_time().sec()) * 1000000000LL +
                        s.sim_time().nsec(), std::memory_order_relaxed);
          })) return 3;
  auto pub = node.Advertise<gz::msgs::Actuators>("/cf_0/command/motor_speed");
  const auto readyBy = Clock::now() + std::chrono::seconds(8);
  std::vector<gz::transport::MessagePublisher> publishers, subscribers;
  while (Clock::now() < readyBy) {
    publishers.clear(); subscribers.clear();
    node.TopicInfo("/cf_0/command/motor_speed", publishers, subscribers);
    if (!subscribers.empty() && simNs.load(std::memory_order_relaxed) >= 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (subscribers.empty() || simNs.load(std::memory_order_relaxed) < 0) return 4;
  auto publish = [&](int active, float speed, const char *label) {
    const auto start = monotonicNs();
    const auto simStart = simNs.load(std::memory_order_relaxed);
    log << "{\"event\":\"phase\",\"label\":\"" << label
        << "\",\"host_monotonic_start_ns\":" << start
        << ",\"sim_time_start_ns\":" << simStart
        << ",\"active_channel\":" << active
        << ",\"commanded_speed_rad_s\":" << speed << "}\n" << std::flush;
    const auto until = Clock::now() + std::chrono::milliseconds(250);
    while (Clock::now() < until) {
      gz::msgs::Actuators m;
      for (int i=0;i<4;++i) m.add_velocity(i == active ? speed : 0.0f);
      if (!pub.Publish(m)) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto end = monotonicNs();
    log << "{\"event\":\"phase_end\",\"label\":\"" << label
        << "\",\"host_monotonic_end_ns\":" << end
        << ",\"sim_time_end_ns\":" << simNs.load(std::memory_order_relaxed)
        << "}\n" << std::flush;
    return true;
  };
  if (!publish(-1, 0, "baseline")) return 5;
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  if (!publish(selected, 500.0f, ("m" + std::to_string(selected+1)).c_str())) return 5;
  return publish(-1, 0, "pulse_end") ? 0 : 5;
}
