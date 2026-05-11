// Offline MTT reference-state smoother.
//
// This executable is intentionally batch/offline, separate from the online
// factor_graph_node. It consumes a compact CSV extracted from one bag and writes
// a best-effort reference trajectory plus quality summary.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using gtsam::symbol_shorthand::X;

namespace {

struct Args {
  std::string input_csv;
  std::string output_csv;
  std::string summary_yaml;
  double icp_sigma_xy{0.03};
  double icp_sigma_yaw{0.03};
  double odom_sigma_xy{0.30};
  double odom_sigma_yaw{0.25};
  double gps_sigma_xy{1.50};
  double robust_k{1.345};
};

struct Row {
  double t{0.0};
  bool has_icp{false};
  double icp_x{0.0};
  double icp_y{0.0};
  double icp_yaw{0.0};
  bool has_odom{false};
  bool odom_is_synthetic{false};
  double odom_x{0.0};
  double odom_y{0.0};
  double odom_yaw{0.0};
  bool has_gps{false};
  double gps_x{0.0};
  double gps_y{0.0};
  double gps_z{0.0};
  bool has_trailer_angle{false};
  double trailer_angle{0.0};
  bool has_trailer_pose{false};
  double trailer_x{0.0};
  double trailer_y{0.0};
  double trailer_z{0.0};
  double quality{0.0};
};

double nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

bool finite(double value)
{
  return std::isfinite(value);
}

double parse_double(const std::string& value, double fallback = nan())
{
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

bool parse_bool(const std::string& value)
{
  return value == "1" || value == "true" || value == "True" || value == "yes";
}

std::vector<std::string> split_csv_line(const std::string& line)
{
  std::vector<std::string> out;
  std::string field;
  bool in_quotes = false;
  for (char c : line) {
    if (c == '"') {
      in_quotes = !in_quotes;
    } else if (c == ',' && !in_quotes) {
      out.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  out.push_back(field);
  return out;
}

std::string get(
  const std::unordered_map<std::string, std::size_t>& index,
  const std::vector<std::string>& fields,
  const std::string& key)
{
  const auto it = index.find(key);
  if (it == index.end() || it->second >= fields.size()) {
    return "";
  }
  return fields[it->second];
}

std::vector<Row> read_rows(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open input CSV: " + path);
  }

  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("input CSV is empty: " + path);
  }

  const auto headers = split_csv_line(line);
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < headers.size(); ++i) {
    index[headers[i]] = i;
  }

  std::vector<Row> rows;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = split_csv_line(line);
    Row row;
    row.t = parse_double(get(index, fields, "t"), 0.0);

    row.icp_x = parse_double(get(index, fields, "icp_x"));
    row.icp_y = parse_double(get(index, fields, "icp_y"));
    row.icp_yaw = parse_double(get(index, fields, "icp_yaw"));
    row.has_icp = parse_bool(get(index, fields, "has_icp")) &&
      finite(row.icp_x) && finite(row.icp_y) && finite(row.icp_yaw);

    row.odom_x = parse_double(get(index, fields, "odom_x"));
    row.odom_y = parse_double(get(index, fields, "odom_y"));
    row.odom_yaw = parse_double(get(index, fields, "odom_yaw"));
    row.has_odom = parse_bool(get(index, fields, "has_odom")) &&
      finite(row.odom_x) && finite(row.odom_y) && finite(row.odom_yaw);
    row.odom_is_synthetic = parse_bool(get(index, fields, "odom_is_synthetic"));

    row.gps_x = parse_double(get(index, fields, "gps_x"));
    row.gps_y = parse_double(get(index, fields, "gps_y"));
    row.gps_z = parse_double(get(index, fields, "gps_z"), 0.0);
    row.has_gps = parse_bool(get(index, fields, "has_gps")) &&
      finite(row.gps_x) && finite(row.gps_y);

    row.trailer_angle = parse_double(get(index, fields, "trailer_angle"));
    row.has_trailer_angle =
      parse_bool(get(index, fields, "has_trailer_angle")) && finite(row.trailer_angle);

    row.trailer_x = parse_double(get(index, fields, "trailer_x"));
    row.trailer_y = parse_double(get(index, fields, "trailer_y"));
    row.trailer_z = parse_double(get(index, fields, "trailer_z"));
    row.has_trailer_pose = parse_bool(get(index, fields, "has_trailer_pose")) &&
      finite(row.trailer_x) && finite(row.trailer_y) && finite(row.trailer_z);

    rows.push_back(row);
  }

  std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.t < b.t; });
  rows.erase(
    std::unique(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
      return std::abs(a.t - b.t) < 1e-6;
    }),
    rows.end());
  return rows;
}

gtsam::Pose3 planar_pose(double x, double y, double yaw)
{
  return gtsam::Pose3(gtsam::Rot3::Rz(yaw), gtsam::Point3(x, y, 0.0));
}

gtsam::Pose3 initial_pose_for_row(const Row& row, const gtsam::Pose3& fallback)
{
  if (row.has_icp) {
    return planar_pose(row.icp_x, row.icp_y, row.icp_yaw);
  }
  if (row.has_odom) {
    return planar_pose(row.odom_x, row.odom_y, row.odom_yaw);
  }
  if (row.has_gps) {
    return gtsam::Pose3(fallback.rotation(), gtsam::Point3(row.gps_x, row.gps_y, row.gps_z));
  }
  return fallback;
}

auto robust_pose_noise(double roll, double pitch, double yaw, double x, double y, double z, double k)
{
  auto base = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << roll, pitch, yaw, x, y, z).finished());
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(k),
    base);
}

Args parse_args(int argc, char** argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto require_value = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + name);
      }
      return argv[++i];
    };
    if (key == "--input") {
      args.input_csv = require_value(key);
    } else if (key == "--output") {
      args.output_csv = require_value(key);
    } else if (key == "--summary") {
      args.summary_yaml = require_value(key);
    } else if (key == "--icp-sigma-xy") {
      args.icp_sigma_xy = std::stod(require_value(key));
    } else if (key == "--icp-sigma-yaw") {
      args.icp_sigma_yaw = std::stod(require_value(key));
    } else if (key == "--odom-sigma-xy") {
      args.odom_sigma_xy = std::stod(require_value(key));
    } else if (key == "--odom-sigma-yaw") {
      args.odom_sigma_yaw = std::stod(require_value(key));
    } else if (key == "--gps-sigma-xy") {
      args.gps_sigma_xy = std::stod(require_value(key));
    } else if (key == "--robust-k") {
      args.robust_k = std::stod(require_value(key));
    } else if (key == "--help" || key == "-h") {
      std::cout
        << "Usage: offline_reference_solver --input measurements.csv --output reference_state.csv "
        << "--summary summary.yaml\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + key);
    }
  }

  if (args.input_csv.empty() || args.output_csv.empty() || args.summary_yaml.empty()) {
    throw std::runtime_error("--input, --output, and --summary are required");
  }
  return args;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    const Args args = parse_args(argc, argv);
    const auto rows = read_rows(args.input_csv);
    if (rows.empty()) {
      throw std::runtime_error("no usable rows in input CSV");
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    std::size_t icp_between_count = 0;
    std::size_t odom_between_count = 0;
    std::size_t gps_prior_count = 0;
    std::size_t synthetic_odom_skipped_count = 0;
    std::size_t trailer_angle_count = 0;
    std::size_t trailer_pose_count = 0;

    gtsam::Pose3 previous_guess = initial_pose_for_row(rows.front(), gtsam::Pose3::Identity());
    for (std::size_t i = 0; i < rows.size(); ++i) {
      previous_guess = initial_pose_for_row(rows[i], previous_guess);
      initial.insert(X(i), previous_guess);
    }

    const auto first_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.2, 0.2, 0.2, 0.5, 0.5, 1.0).finished());
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), initial.at<gtsam::Pose3>(X(0)), first_noise));

    const auto icp_noise = robust_pose_noise(10.0, 10.0, args.icp_sigma_yaw,
      args.icp_sigma_xy, args.icp_sigma_xy, 10.0, args.robust_k);
    const auto odom_noise = robust_pose_noise(10.0, 10.0, args.odom_sigma_yaw,
      args.odom_sigma_xy, args.odom_sigma_xy, 10.0, args.robust_k);
    const auto gps_noise = robust_pose_noise(99.0, 99.0, 99.0,
      args.gps_sigma_xy, args.gps_sigma_xy, 3.0, args.robust_k);

    for (std::size_t i = 1; i < rows.size(); ++i) {
      const auto& prev = rows[i - 1];
      const auto& curr = rows[i];

      if (prev.has_icp && curr.has_icp) {
        const auto a = planar_pose(prev.icp_x, prev.icp_y, prev.icp_yaw);
        const auto b = planar_pose(curr.icp_x, curr.icp_y, curr.icp_yaw);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(i - 1), X(i), a.between(b), icp_noise));
        ++icp_between_count;
      }

      if (prev.has_odom && curr.has_odom) {
        if (!prev.odom_is_synthetic && !curr.odom_is_synthetic) {
          const auto a = planar_pose(prev.odom_x, prev.odom_y, prev.odom_yaw);
          const auto b = planar_pose(curr.odom_x, curr.odom_y, curr.odom_yaw);
          graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(i - 1), X(i), a.between(b), odom_noise));
          ++odom_between_count;
        } else {
          ++synthetic_odom_skipped_count;
        }
      }

      if (curr.has_gps) {
        const auto gps_pose = gtsam::Pose3(
          initial.at<gtsam::Pose3>(X(i)).rotation(),
          gtsam::Point3(curr.gps_x, curr.gps_y, curr.gps_z));
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(i), gps_pose, gps_noise));
        ++gps_prior_count;
      }

      if (curr.has_trailer_angle) {
        ++trailer_angle_count;
      }
      if (curr.has_trailer_pose) {
        ++trailer_pose_count;
      }
    }

    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("ERROR");
    params.setMaxIterations(100);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    const auto result = optimizer.optimize();
    const double final_error = graph.error(result);

    std::ofstream csv(args.output_csv);
    if (!csv) {
      throw std::runtime_error("cannot open output CSV: " + args.output_csv);
    }
    csv << std::setprecision(15);
    csv << "t,x,y,z,qx,qy,qz,qw,source_quality,has_icp,has_gps,has_real_odom,"
        << "has_trailer_angle,trailer_angle,has_trailer_pose,trailer_x,trailer_y,trailer_z\n";
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const auto pose = result.at<gtsam::Pose3>(X(i));
      const auto q = pose.rotation().toQuaternion();
      const auto p = pose.translation();
      const auto& row = rows[i];

      double quality = 0.0;
      if (row.has_icp) quality += 0.55;
      if (row.has_gps) quality += 0.20;
      if (row.has_odom && !row.odom_is_synthetic) quality += 0.15;
      if (row.has_trailer_angle || row.has_trailer_pose) quality += 0.10;
      quality = std::min(1.0, quality);

      csv << row.t << ','
          << p.x() << ',' << p.y() << ',' << p.z() << ','
          << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
          << quality << ','
          << (row.has_icp ? 1 : 0) << ','
          << (row.has_gps ? 1 : 0) << ','
          << (row.has_odom && !row.odom_is_synthetic ? 1 : 0) << ','
          << (row.has_trailer_angle ? 1 : 0) << ','
          << (row.has_trailer_angle ? row.trailer_angle : nan()) << ','
          << (row.has_trailer_pose ? 1 : 0) << ','
          << (row.has_trailer_pose ? row.trailer_x : nan()) << ','
          << (row.has_trailer_pose ? row.trailer_y : nan()) << ','
          << (row.has_trailer_pose ? row.trailer_z : nan()) << '\n';
    }

    std::ofstream summary(args.summary_yaml);
    if (!summary) {
      throw std::runtime_error("cannot open summary YAML: " + args.summary_yaml);
    }
    summary << "status: ok\n";
    summary << "input_csv: " << args.input_csv << "\n";
    summary << "output_csv: " << args.output_csv << "\n";
    summary << "row_count: " << rows.size() << "\n";
    summary << "factor_count: " << graph.size() << "\n";
    summary << "initial_error: " << graph.error(initial) << "\n";
    summary << "final_error: " << final_error << "\n";
    summary << "icp_between_count: " << icp_between_count << "\n";
    summary << "real_odom_between_count: " << odom_between_count << "\n";
    summary << "synthetic_odom_between_skipped_count: " << synthetic_odom_skipped_count << "\n";
    summary << "gps_prior_count: " << gps_prior_count << "\n";
    summary << "trailer_angle_rows: " << trailer_angle_count << "\n";
    summary << "trailer_pose_rows: " << trailer_pose_count << "\n";

    std::cout << "offline_reference_solver OK\n";
    std::cout << "  rows: " << rows.size() << "\n";
    std::cout << "  factors: " << graph.size() << "\n";
    std::cout << "  output: " << args.output_csv << "\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "offline_reference_solver ERROR: " << exc.what() << "\n";
    return 1;
  }
}
