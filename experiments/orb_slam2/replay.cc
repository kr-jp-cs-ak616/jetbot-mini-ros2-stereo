#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include "System.h"

int main(int argc, char** argv)
{
    if (argc != 5) {
        std::cerr << "Usage: jetbot_orb_replay VOCAB SETTINGS RECTIFIED_SEQUENCE OUTPUT\n";
        return 2;
    }
    namespace fs = std::filesystem;
    const fs::path input(argv[3]), output(argv[4]);
    std::ifstream pairs(input / "pairs.tsv");
    if (!pairs || !fs::is_directory(output) || fs::exists(output / "trajectory.tum")) {
        std::cerr << "Expected a sequence with pairs.tsv and a fresh output directory\n";
        return 2;
    }
    cv::setNumThreads(2);
    ORB_SLAM2::System slam(argv[1], argv[2], ORB_SLAM2::System::STEREO, false);
    std::ofstream status(output / "tracking.tsv");
    status << "timestamp\tstate\ttracking_ms\n";
    status.precision(9);
    status << std::fixed;
    std::string name;
    double time = 0;
    int total = 0, tracked = 0;
    bool failed = false;
    while (pairs >> time >> name) {
        const auto left = cv::imread((input / "left" / name).string(), cv::IMREAD_GRAYSCALE);
        const auto right = cv::imread((input / "right" / name).string(), cv::IMREAD_GRAYSCALE);
        if (left.size() != cv::Size(640,480) || right.size() != left.size()) {
            std::cerr << "Missing or invalid stereo input at " << name << '\n';
            failed = true;
            break;
        }
        const auto begin = std::chrono::steady_clock::now();
        const auto pose = slam.TrackStereo(left,right,time);
        const double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now() - begin).count();
        const bool ok = !pose.empty() && slam.GetTrackingState() == ORB_SLAM2::Tracking::OK;
        tracked += ok;
        ++total;
        status << time << '\t' << slam.GetTrackingState() << '\t' << ms << '\n';
        // Allow local mapping and loop closing to work between replayed frames.
        if (ms < 40) std::this_thread::sleep_for(std::chrono::duration<double,std::milli>(40 - ms));
    }
    slam.Shutdown();
    if (tracked > 0) {
        slam.SaveTrajectoryTUM((output / "trajectory.tum").string());
        slam.SaveMapPoints((output / "map-points.xyz").string());
        slam.SaveKeyFrameTrajectoryTUM((output / "keyframes.tum").string());
    }
    std::cout << "Tracked " << tracked << " / " << total << '\n';
    return failed || tracked == 0;
}
