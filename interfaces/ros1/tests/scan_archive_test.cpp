#include <bievr_lio/scan_archive.h>
#include <bievr_lio/pipeline.h>
#include <chrono>
#include <iostream>

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("bievr-scan-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto path = directory / "scans.bin";
  auto require = [](bool condition) {
    if (!condition) throw std::runtime_error("Scan archive assertion failed.");
  };
  try {
    {
      bievr::ScanArchiveWriter writer(path);
      bievr::IntensityPointcloud cloud;
      cloud.resize(2);
      cloud[0] << 1.25, -2.5, 3.75, 42;
      cloud[1] << -1, 2, -3, 7;
      for (uint64_t i = 0; i < 100; ++i) writer.add(1700000000123456789ULL + i, cloud);
      require(writer.finish());
      require(writer.finish());
    }
    bievr::ScanArchiveReader reader(path.string());
    uint64_t stamp;
    bievr::IntensityPointcloud cloud;
    for (uint64_t i = 0; i < 100; ++i) {
      require(reader.next(stamp, cloud));
      require(stamp == 1700000000123456789ULL + i && cloud.size() == 2);
      require(cloud[0][0] == 1.25 && cloud[0][1] == -2.5 && cloud[0][3] == 42);
      require(cloud[1][2] == -3 && cloud[1][3] == 7);
    }
    require(!reader.next(stamp, cloud));
    bool refused = false;
    try { bievr::ScanArchiveWriter duplicate(path); }
    catch (const std::exception&) { refused = true; }
    require(refused);
    // Exercise the frontend hook with non-identity LiDAR->IMU calibration.
    bievr::Pipeline::Config config;
    config.accumulate_map = false;
    config.record_gnss_scans = true;
    config.publish_path = false;
    config.imu.t_init = 0.01;
    config.T_I_L.translation() = bievr::V3(1, 2, 3);
    config.map_save_path = (directory / "pipeline/map.pcd").string();
    {
      bievr::Pipeline pipeline(config);
      pipeline.registerPublisher<bievr::IntensityPointcloud>(
          [](const auto&, const auto&, const auto&, const auto&) {});
      pipeline.registerPublisher<bievr::Odometry>(
          [](const auto&, const auto&, const auto&, const auto&) {});
      pipeline.registerPublisher<bievr::V3>(
          [](const auto&, const auto&, const auto&, const auto&) {});
      for (uint64_t frame = 0; frame < 2; ++frame) {
        bievr::StampedIntensityPointcloud input;
        input.stamp = 1000000000ULL + frame * 100000000ULL;
        input.end_stamp = input.stamp + 100000000ULL;
        input.resize(100);
        for (size_t i = 0; i < input.size(); ++i)
          input[i] << 5.0 + (i % 10) * 0.1, (i / 10) * 0.1, 0, 0.05, 42;
        std::vector<bievr::ImuMeasurement> imu;
        for (uint64_t i = 0; i <= 10; ++i)
          imu.push_back({input.stamp + i * 10000000ULL,
                         bievr::V3(0, 0, bievr::kGMagnitude), bievr::V3::Zero()});
        pipeline.processFrame(imu, input);
      }
      require(pipeline.saveResults());
    }
    bievr::ScanArchiveReader frontend((directory / "pipeline/gnss/scans.bin").string());
    for (uint64_t i = 0; i < 2; ++i) {
      require(frontend.next(stamp, cloud));
      require(stamp == 1100000000ULL + i * 100000000ULL && cloud.size() == 100);
      require(std::abs(cloud[0][0] - 6) < 1e-4 && std::abs(cloud[0][1] - 2) < 1e-4 &&
              std::abs(cloud[0][2] - 3) < 1e-4 && cloud[0][3] == 42);
    }
    require(!frontend.next(stamp, cloud));
    std::filesystem::remove_all(directory);
    std::cout << "PASS: scan archive worker drain, order, precise stamps, intensity, overwrite protection, frontend hook and extrinsics\n";
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
