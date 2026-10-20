#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <google/protobuf/util/json_util.h>

using Clock = std::chrono::steady_clock;
std::int64_t monotonicNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch()).count();
}
template <typename Message>
bool writeJson(std::ofstream &out, std::mutex &mutex, const Message &msg) {
  std::string json;
  auto status=google::protobuf::util::MessageToJsonString(msg,&json);
  if (!status.ok()) return false;
  std::lock_guard<std::mutex> lock(mutex);
  out << json << '\n';
  return static_cast<bool>(out);
}
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const int selected = std::stoi(argv[1]);
  if (selected < 0 || selected > 3) return 2;
  const std::filesystem::path outdir(argv[2]);
  std::filesystem::create_directories(outdir);
  std::ofstream events(outdir/"publisher-events.jsonl");
  std::ofstream odom(outdir/"odometry.jsonl");
  std::ofstream stats(outdir/"world-stats.jsonl");
  std::ofstream motors(outdir/"commands.jsonl");
  if (!events || !odom || !stats || !motors) return 2;
  std::mutex eventsMu,odomMu,statsMu,motorsMu;
  gz::transport::Node node;
  std::atomic<std::int64_t> simNs{-1};
  if (!node.Subscribe<gz::msgs::WorldStatistics>(
          "/world/crazysim_default/stats",
          [&simNs,&stats,&statsMu](const gz::msgs::WorldStatistics &s) {
            simNs.store(static_cast<std::int64_t>(s.sim_time().sec())*1000000000LL+s.sim_time().nsec(),std::memory_order_relaxed);
            writeJson(stats,statsMu,s);
          })) return 3;
  if (!node.Subscribe<gz::msgs::Odometry>("/cf_0/odom",
          [&odom,&odomMu](const gz::msgs::Odometry &m) { writeJson(odom,odomMu,m); })) return 3;
  if (!node.Subscribe<gz::msgs::Actuators>("/cf_0/command/motor_speed",
          [&motors,&motorsMu](const gz::msgs::Actuators &m) {
            std::lock_guard<std::mutex> lock(motorsMu);
            motors << "{\"capture_monotonic_ns\":" << monotonicNs() << ",\"velocity\":[";
            for(int i=0;i<m.velocity_size();++i){if(i)motors<<',';motors<<m.velocity(i);} motors << "]}\n";
          })) return 3;
  auto pub=node.Advertise<gz::msgs::Actuators>("/cf_0/command/motor_speed");
  const auto readyBy=Clock::now()+std::chrono::seconds(8);
  std::vector<gz::transport::MessagePublisher> publishers,subscribers;
  while(Clock::now()<readyBy){
    publishers.clear();subscribers.clear();node.TopicInfo("/cf_0/command/motor_speed",publishers,subscribers);
    if(!subscribers.empty()&&simNs.load(std::memory_order_relaxed)>=0)break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if(subscribers.empty()||simNs.load(std::memory_order_relaxed)<0)return 4;
  auto phase=[&](int active,float speed,const std::string &label,int durationMs){
    {std::lock_guard<std::mutex> lock(eventsMu);events<<"{\"event\":\"phase\",\"label\":\""<<label<<"\",\"host_monotonic_ns\":"<<monotonicNs()<<",\"sim_time_ns\":"<<simNs.load()<<",\"active_channel\":"<<active<<",\"speed_rad_s\":"<<speed<<"}\n"<<std::flush;}
    const auto until=Clock::now()+std::chrono::milliseconds(durationMs);
    while(Clock::now()<until){gz::msgs::Actuators msg;for(int i=0;i<4;++i)msg.add_velocity(i==active?speed:0.0f);if(!pub.Publish(msg))return false;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    {std::lock_guard<std::mutex> lock(eventsMu);events<<"{\"event\":\"phase_end\",\"label\":\""<<label<<"\",\"host_monotonic_ns\":"<<monotonicNs()<<",\"sim_time_ns\":"<<simNs.load()<<"}\n"<<std::flush;}
    return true;
  };
  if(!phase(-1,0,"baseline",200)||!phase(selected,100.0f,"m"+std::to_string(selected+1),200)||!phase(-1,0,"pulse_end",400))return 5;
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  node.Unsubscribe("/world/crazysim_default/stats");node.Unsubscribe("/cf_0/odom");node.Unsubscribe("/cf_0/command/motor_speed");
  {std::lock_guard<std::mutex> lock(odomMu);odom.flush();}
  {std::lock_guard<std::mutex> lock(statsMu);stats.flush();}
  {std::lock_guard<std::mutex> lock(motorsMu);motors.flush();}
  events.flush();
  return 0;
}
