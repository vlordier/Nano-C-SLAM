#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/transport/SubscribeOptions.hh>
#include <google/protobuf/util/json_util.h>

namespace {
volatile std::sig_atomic_t running = 1;
std::atomic<std::uint64_t> odometryCount{0};
std::atomic<std::uint64_t> imuCount{0};
std::atomic<std::uint64_t> motorCount{0};
std::atomic<std::uint64_t> worldStatsCount{0};
std::atomic<std::uint64_t> jointStateCount{0};

void stop(int) {
    running = 0;
}

template <typename Message>
std::function<void(const Message &)> jsonRecorder(std::ofstream &output,
                                                  std::mutex &outputMutex,
                                                  std::atomic<std::uint64_t> &count) {
    return [&output, &outputMutex, &count](const Message &message) {
        const auto arrivalWallNs =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        const auto arrivalMonotonicNs =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        std::string serialized;
        const auto status =
            google::protobuf::util::MessageToJsonString(message, &serialized);
        if (!status.ok()) {
            std::cerr << "failed to encode sampled protobuf as JSON: "
                      << status.message() << '\n';
            return;
        }
        const auto objectStart = serialized.find('{');
        if (objectStart == std::string::npos) {
            std::cerr << "encoded protobuf JSON is not an object\n";
            return;
        }
        const std::string arrivalFields =
            "\"capture_unix_time_ns\":" + std::to_string(arrivalWallNs) +
            ",\"capture_monotonic_time_ns\":" + std::to_string(arrivalMonotonicNs) +
            ',';
        serialized.insert(objectStart + 1, arrivalFields);
        std::lock_guard<std::mutex> lock(outputMutex);
        output << serialized << '\n';
        count.fetch_add(1, std::memory_order_relaxed);
    };
}

bool prepareOutput(const std::filesystem::path &path, std::ofstream &output) {
    if (std::filesystem::exists(path)) {
        std::cerr << "refusing to overwrite capture file: " << path << '\n';
        return false;
    }
    output.open(path);
    if (!output) {
        std::cerr << "failed to create capture file: " << path << '\n';
        return false;
    }
    return true;
}

template <typename Message>
bool subscribeAndReport(gz::transport::Node &node, const std::string &topic,
                        const std::string &label,
                        std::function<void(const Message &)> callback,
                        const gz::transport::SubscribeOptions &options,
                        const std::chrono::steady_clock::time_point &deadline) {
    const bool subscribed = node.Subscribe<Message>(topic, std::move(callback), options);
    std::vector<gz::transport::MessagePublisher> publishers;
    std::vector<gz::transport::MessagePublisher> subscribers;
    const auto messageType = Message().GetTypeName();
    const std::string expectedType(messageType.data(), messageType.size());
    std::size_t matchingPublishers = 0;
    bool discovered = false;
    do {
        publishers.clear();
        subscribers.clear();
        discovered = node.TopicInfo(topic, publishers, subscribers);
        matchingPublishers = 0;
        for (const auto &publisher : publishers) {
            if (publisher.MsgTypeName() == expectedType) {
                ++matchingPublishers;
            }
        }
        if (!subscribed || matchingPublishers > 0 ||
            std::chrono::steady_clock::now() >= deadline) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (true);
    std::cerr << "topic=" << label << " name=" << topic
              << " subscribed=" << (subscribed ? "yes" : "no")
              << " discovered=" << (discovered ? "yes" : "no")
              << " publishers=" << publishers.size();
    for (const auto &publisher : publishers) {
        std::cerr << " [type=" << publisher.MsgTypeName()
                  << " addr=" << publisher.Addr() << ']';
    }
    std::cerr << " expected_type=" << expectedType
              << " matching_publishers=" << matchingPublishers;
    if (matchingPublishers == 0) {
        std::cerr << " publisher_discovery=timed_out";
    }
    std::cerr << '\n';
    return subscribed && discovered && matchingPublishers > 0;
}
} // namespace

int main(int argc, char **argv) {
    const bool requireJointState = argc == 4 && std::string(argv[3]) == "--joint-state";
    if ((argc != 3 && argc != 4) || (argc == 4 && !requireJointState)) {
        std::cerr << "usage: crazysim_flight_sampler DURATION_SECONDS OUTPUT_DIR [--joint-state]\n";
        return 2;
    }

    char *end = nullptr;
    const long durationSeconds = std::strtol(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0' || durationSeconds < 1) {
        std::cerr << "duration-seconds must be a positive integer\n";
        return 2;
    }

    const std::filesystem::path outputDir(argv[2]);
    std::error_code error;
    std::filesystem::create_directories(outputDir, error);
    if (error) {
        std::cerr << "failed to create output directory: " << error.message() << '\n';
        return 2;
    }

    std::ofstream odometryOutput;
    std::ofstream imuOutput;
    std::ofstream motorOutput;
    std::ofstream worldStatsOutput;
    std::ofstream jointStateOutput;
    std::mutex odometryMutex;
    std::mutex imuMutex;
    std::mutex motorMutex;
    std::mutex worldStatsMutex;
    std::mutex jointStateMutex;
    if (!prepareOutput(outputDir / "odometry.jsonl", odometryOutput) ||
        !prepareOutput(outputDir / "imu.jsonl", imuOutput) ||
        !prepareOutput(outputDir / "motors.jsonl", motorOutput) ||
        !prepareOutput(outputDir / "world-stats.jsonl", worldStatsOutput) ||
        (requireJointState &&
         !prepareOutput(outputDir / "joint-state.jsonl", jointStateOutput))) {
        return 2;
    }

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    gz::transport::Node node;
    const char *gazeboIp = std::getenv("GZ_IP");
    std::cerr << "transport partition=" << node.Options().Partition()
              << " GZ_IP=" << (gazeboIp == nullptr ? "unset" : gazeboIp) << '\n';
    gz::transport::SubscribeOptions odometryOptions;
    // Do not throttle state or actuator topics at the subscriber boundary:
    // transport scheduling can alias a nominal rate cap and create artificial
    // timestamp gaps. This sampler is intended to preserve source cadence.
    gz::transport::SubscribeOptions imuOptions;
    gz::transport::SubscribeOptions motorOptions;
    gz::transport::SubscribeOptions worldStatsOptions;
    worldStatsOptions.SetMsgsPerSec(10);
    const auto publisherDiscoveryDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(15);

    const bool odometrySubscribed = subscribeAndReport<gz::msgs::Odometry>(
        node, "/cf_0/odom", "odometry",
        jsonRecorder<gz::msgs::Odometry>(odometryOutput, odometryMutex,
                                         odometryCount),
        odometryOptions, publisherDiscoveryDeadline);
    const bool imuSubscribed = subscribeAndReport<gz::msgs::IMU>(
        node, "/cf_0/imu", "imu",
        jsonRecorder<gz::msgs::IMU>(imuOutput, imuMutex, imuCount), imuOptions,
        publisherDiscoveryDeadline);
    const bool motorSubscribed = subscribeAndReport<gz::msgs::Actuators>(
        node, "/cf_0/command/motor_speed", "motor",
        jsonRecorder<gz::msgs::Actuators>(motorOutput, motorMutex, motorCount),
        motorOptions, publisherDiscoveryDeadline);
    const bool worldStatsSubscribed = subscribeAndReport<gz::msgs::WorldStatistics>(
        node, "/world/crazysim_default/stats", "world_stats",
        jsonRecorder<gz::msgs::WorldStatistics>(worldStatsOutput, worldStatsMutex,
                                                worldStatsCount),
        worldStatsOptions, publisherDiscoveryDeadline);
    bool jointStateSubscribed = true;
    if (requireJointState) {
        gz::transport::SubscribeOptions jointStateOptions;
        jointStateSubscribed = subscribeAndReport<gz::msgs::Model>(
            node, "/world/crazysim_default/model/crazyflie_0/joint_state",
            "joint_state",
            jsonRecorder<gz::msgs::Model>(jointStateOutput, jointStateMutex,
                                          jointStateCount),
            jointStateOptions, publisherDiscoveryDeadline);
    }
    const bool subscribed = odometrySubscribed && imuSubscribed && motorSubscribed &&
                            worldStatsSubscribed && jointStateSubscribed;
    if (!subscribed) {
        std::cerr << "failed to subscribe to one or more CrazySim topics\n";
        return 3;
    }

    std::cerr << "sampling CrazySim odometry (publisher 200 Hz, uncapped), IMU "
                 "(uncapped), motor commands (uncapped), world stats (10 Hz)"
              << (requireJointState ? ", and joint state" : "") << " as "
                 "compact JSONL for "
              << durationSeconds << " seconds\n";
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(durationSeconds);
    while (running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const bool odometryUnsubscribed = node.Unsubscribe("/cf_0/odom");
    const bool imuUnsubscribed = node.Unsubscribe("/cf_0/imu");
    const bool motorUnsubscribed = node.Unsubscribe("/cf_0/command/motor_speed");
    const bool worldStatsUnsubscribed =
        node.Unsubscribe("/world/crazysim_default/stats");
    const bool jointStateUnsubscribed = !requireJointState ||
        node.Unsubscribe("/world/crazysim_default/model/crazyflie_0/joint_state");
    {
        std::lock_guard<std::mutex> lock(odometryMutex);
        odometryOutput.flush();
    }
    {
        std::lock_guard<std::mutex> lock(imuMutex);
        imuOutput.flush();
    }
    {
        std::lock_guard<std::mutex> lock(motorMutex);
        motorOutput.flush();
    }
    {
        std::lock_guard<std::mutex> lock(worldStatsMutex);
        worldStatsOutput.flush();
    }
    if (requireJointState) {
        std::lock_guard<std::mutex> lock(jointStateMutex);
        jointStateOutput.flush();
    }
    std::cerr << "captured odometry=" << odometryCount.load()
              << " imu=" << imuCount.load() << " motor=" << motorCount.load()
              << " world_stats=" << worldStatsCount.load();
    if (requireJointState) {
        std::cerr << " joint_state=" << jointStateCount.load();
    }
    std::cerr << '\n';
    if (odometryCount.load() == 0 || imuCount.load() == 0 ||
        motorCount.load() == 0 || worldStatsCount.load() == 0) {
        std::cerr << "no callbacks for:";
        if (odometryCount.load() == 0) std::cerr << " odometry";
        if (imuCount.load() == 0) std::cerr << " imu";
        if (motorCount.load() == 0) std::cerr << " motor";
        if (worldStatsCount.load() == 0) std::cerr << " world_stats";
        if (requireJointState && jointStateCount.load() == 0) {
            std::cerr << " joint_state";
        }
        std::cerr << "; match both GZ_PARTITION and GZ_IP with Gazebo, then verify "
                     "publisher type/address and transport reachability\n";
    }
    return odometryUnsubscribed && imuUnsubscribed && motorUnsubscribed &&
                   worldStatsUnsubscribed && odometryCount.load() > 0 &&
                   imuCount.load() > 0 && motorCount.load() > 0 &&
                   worldStatsCount.load() > 0 && jointStateUnsubscribed &&
                   (!requireJointState || jointStateCount.load() > 0)
               ? 0
               : 4;
}
