#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/SubscribeOptions.hh>

namespace {
volatile std::sig_atomic_t running = 1;
std::atomic<std::uint64_t> sampleCount{0};

void stop(int) {
    running = 0;
}

void printTime(const char *name, const gz::msgs::Time &time) {
    std::cout << name << " { sec: " << time.sec() << " nsec: " << time.nsec() << " }\n";
}
} // namespace

int main(int argc, char **argv) {
    const int durationSeconds = argc > 1 ? std::atoi(argv[1]) : 120;
    const std::string topic = argc > 2 ? argv[2] : "/stats";
    if (durationSeconds < 1) {
        std::cerr << "duration-seconds must be positive\n";
        return 2;
    }

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    gz::transport::Node node;
    gz::transport::SubscribeOptions options;
    options.SetMsgsPerSec(1);
    std::function<void(const gz::msgs::WorldStatistics &)> callback =
        [](const gz::msgs::WorldStatistics &stats) {
            sampleCount.fetch_add(1, std::memory_order_relaxed);
            printTime("sim_time", stats.sim_time());
            printTime("real_time", stats.real_time());
            std::cout << "iterations: " << stats.iterations() << "\n"
                      << "real_time_factor: " << stats.real_time_factor() << "\n\n"
                      << std::flush;
        };
    const bool subscribed =
        node.Subscribe<gz::msgs::WorldStatistics>(topic, callback, options);
    if (!subscribed) {
        std::cerr << "failed to subscribe to " << topic << "\n";
        return 3;
    }

    std::cerr << "sampling " << topic << " at most once per second for "
              << durationSeconds << " seconds\n";
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(durationSeconds);
    while (running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const auto capturedSamples = sampleCount.load(std::memory_order_relaxed);
    std::cerr << "captured_samples=" << capturedSamples << "\n";
    return capturedSamples >= 2 ? 0 : 4;
}
