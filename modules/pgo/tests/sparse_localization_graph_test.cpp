#include "bievr_pgo/sparse_localization_graph.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bievr::Transform pose(double x, double y, double z, double roll, double pitch,
                      double yaw) {
  const bievr::Rotation rotation =
      (Eigen::AngleAxisd(yaw, bievr::V3::UnitZ()) *
       Eigen::AngleAxisd(pitch, bievr::V3::UnitY()) *
       Eigen::AngleAxisd(roll, bievr::V3::UnitX()))
          .toRotationMatrix();
  return bievr::Transform(rotation, bievr::V3(x, y, z));
}

bievr::V3 rpy(const bievr::Rotation& rotation) {
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  return bievr::V3(std::atan2(rotation(2, 1), rotation(2, 2)), pitch,
                   std::atan2(rotation(1, 0), rotation(0, 0)));
}

void require(bool condition, const char* message) {
  if (condition) return;
  std::cerr << message << '\n';
  std::exit(EXIT_FAILURE);
}

}  // namespace

int main() {
  bievr::SparseLocalizationGraph::Config config;
  config.prior_translation_sigma_m = 0.2;
  config.prior_rotation_sigma_deg = 1.0;
  config.robust_huber_delta = 0.0;
  bievr::SparseLocalizationGraph graph(config);

  graph.setInitialCorrection(bievr::Transform::Identity());
  require(graph.addKeyframe(100, pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
          "failed to add keyframe 0");
  require(graph.addKeyframe(200, pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
          "failed to add keyframe 1");
  require(graph.addKeyframe(300, pose(20.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
          "failed to add keyframe 2");

  // A planar observation deliberately contains a large vertical component;
  // the factor must not pass it into z/roll/pitch.
  require(graph.addPlanarMeasurement(
              200, pose(1.0, 2.0, 50.0, 0.4, -0.3, 0.05), 0.05, 0.01),
          "failed to add planar factor");
  const bievr::Transform after_planar = *graph.correctionAt(200);
  const bievr::V3 planar_rpy = rpy(after_planar.rotation());
  require(std::abs(after_planar.translation().z()) < 1.0e-6,
          "planar factor changed z");
  require(std::abs(planar_rpy.x()) < 1.0e-6, "planar factor changed roll");
  require(std::abs(planar_rpy.y()) < 1.0e-6, "planar factor changed pitch");
  require(after_planar.translation().head<2>().norm() > 0.1,
          "planar factor did not affect x/y");

  const double planar_x = after_planar.translation().x();
  const double planar_y = after_planar.translation().y();
  const double planar_yaw = planar_rpy.z();

  // A vertical observation deliberately contains a very different planar
  // transform; only z/roll/pitch may affect the graph.
  require(graph.addVerticalMeasurement(
              300, pose(100.0, -80.0, 1.5, 0.02, -0.03, 1.0), 0.05,
              0.005, 0.005),
          "failed to add vertical factor");
  const bievr::Transform after_vertical = *graph.correctionAt(300);
  const bievr::V3 vertical_rpy = rpy(after_vertical.rotation());
  require(std::abs(after_vertical.translation().z()) > 0.1,
          "vertical factor did not affect z");
  require(std::hypot(vertical_rpy.x(), vertical_rpy.y()) > 1.0e-3,
          "vertical factor did not affect roll/pitch");
  require(std::abs(after_vertical.translation().x() - planar_x) < 0.2,
          "vertical factor changed x");
  require(std::abs(after_vertical.translation().y() - planar_y) < 0.2,
          "vertical factor changed y");
  require(std::abs(vertical_rpy.z() - planar_yaw) < 0.02,
          "vertical factor changed yaw");

  // A rail tangent along +x must constrain cross-track y, z and all three
  // angles, but a deliberately huge x measurement must remain unobservable.
  const double before_rail_x = after_vertical.translation().x();
  const double before_rail_y = after_vertical.translation().y();
  const double before_rail_yaw = vertical_rpy.z();
  require(graph.addRailMeasurement(
              300, pose(100.0, 2.0, 2.5, 0.04, -0.05, 0.12),
              bievr::V3::UnitX(), 0.05, 0.05, 0.005, 0.005, 0.01),
          "failed to add five-DoF rail factor");
  const bievr::Transform after_rail = *graph.correctionAt(300);
  const bievr::V3 rail_rpy = rpy(after_rail.rotation());
  require(std::abs(after_rail.translation().x() - before_rail_x) < 0.2,
          "rail factor constrained along-track translation");
  require(std::abs(after_rail.translation().y() - before_rail_y) > 0.1,
          "rail factor did not constrain cross-track translation");
  require(std::abs(after_rail.translation().z() - after_vertical.translation().z()) > 0.1,
          "rail factor did not constrain z");
  require(std::abs(rail_rpy.z() - before_rail_yaw) > 1.0e-3,
          "rail factor did not constrain yaw");

  return 0;
}
