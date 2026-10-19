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

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/SubscribeOptions.hh>

namespace {
volatile std::sig_atomic_t running = 1;
std::atomic<std::uint64_t> odometryCount{0};
std::atomic<std::uint64_t> imuCount{0};
std::atomic<std::uint64_t> motorCount{0};
std::atomic<std::uint64_t> worldStatsCount{0};

void stop(int) {
    running = 0;
}

template <typename Message>
std::function<void(const Message &)> textRecorder(std::ofstream &output,
                                                  std::mutex &outputMutex,
                                                  std::atomic<std::uint64_t> &count) {
    return [&output, &outputMutex, &count](const Message &message) {
        std::lock_guard<std::mutex> lock(outputMutex);
        output << message.DebugString() << '\n';
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
} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "usage: crazysim_flight_sampler DURATION_SECONDS OUTPUT_DIR\n";
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
    std::mutex odometryMutex;
    std::mutex imuMutex;
    std::mutex motorMutex;
    std::mutex worldStatsMutex;
    if (!prepareOutput(outputDir / "odometry.txt", odometryOutput) ||
        !prepareOutput(outputDir / "imu.txt", imuOutput) ||
        !prepareOutput(outputDir / "motors.txt", motorOutput) ||
        !prepareOutput(outputDir / "world-stats.txt", worldStatsOutput)) {
        return 2;
    }

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    gz::transport::Node node;
    gz::transport::SubscribeOptions odometryOptions;
    odometryOptions.SetMsgsPerSec(100);
    gz::transport::SubscribeOptions imuOptions;
    imuOptions.SetMsgsPerSec(50);
    gz::transport::SubscribeOptions motorOptions;
    motorOptions.SetMsgsPerSec(50);
    gz::transport::SubscribeOptions worldStatsOptions;
    worldStatsOptions.SetMsgsPerSec(2);

    const bool subscribed =
        node.Subscribe<gz::msgs::Odometry>(
            "/cf_0/odom",
            textRecorder<gz::msgs::Odometry>(odometryOutput, odometryMutex,
                                             odometryCount),
            odometryOptions) &&
        node.Subscribe<gz::msgs::IMU>(
            "/cf_0/imu", textRecorder<gz::msgs::IMU>(imuOutput, imuMutex, imuCount),
            imuOptions) &&
        node.Subscribe<gz::msgs::Actuators>(
            "/cf_0/command/motor_speed",
            textRecorder<gz::msgs::Actuators>(motorOutput, motorMutex, motorCount),
            motorOptions) &&
        node.Subscribe<gz::msgs::WorldStatistics>(
            "/world/crazysim_default/stats",
            textRecorder<gz::msgs::WorldStatistics>(worldStatsOutput, worldStatsMutex,
                                                    worldStatsCount),
            worldStatsOptions);
    if (!subscribed) {
        std::cerr << "failed to subscribe to one or more CrazySim topics\n";
        return 3;
    }

    std::cerr << "sampling CrazySim odometry (100 Hz), IMU (50 Hz), motors "
                 "(50 Hz), and world stats (2 Hz) for "
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
    std::cerr << "captured odometry=" << odometryCount.load()
              << " imu=" << imuCount.load() << " motor=" << motorCount.load()
              << " world_stats=" << worldStatsCount.load() << '\n';
    return odometryUnsubscribed && imuUnsubscribed && motorUnsubscribed &&
                   worldStatsUnsubscribed && odometryCount.load() > 0 &&
                   imuCount.load() > 0 && motorCount.load() > 0 &&
                   worldStatsCount.load() > 0
               ? 0
               : 4;
}
