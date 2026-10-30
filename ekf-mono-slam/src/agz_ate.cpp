#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>

#include "filter/ekf.h"

namespace {

  std::string trim(std::string value) {
    const auto start = value.find_first_not_of(" \t\r");
    if (start == std::string::npos) {
      return {};
    }
    const auto end = value.find_last_not_of(" \t\r");
    value = value.substr(start, end - start + 1);
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }
    return value;
  }

  std::map<std::string, std::string> load_yaml(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
      throw std::runtime_error("cannot open " + path);
    }
    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(input, line)) {
      const auto comment = line.find('#');
      if (comment != std::string::npos) {
        line = line.substr(0, comment);
      }
      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      const std::string key = trim(line.substr(0, colon));
      const std::string value = trim(line.substr(colon + 1));
      if (!key.empty() && !value.empty()) {
        values[key] = value;
      }
    }
    return values;
  }

  double required_double(
    const std::map<std::string, std::string>& values, const std::string& key
  ) {
    const auto found = values.find(key);
    if (found == values.end()) {
      throw std::runtime_error("missing " + key);
    }
    return std::stod(found->second);
  }

  SlamConfig config_from_yaml(const std::string& path) {
    const auto values = load_yaml(path);
    SlamConfig config;
    config.camera.px = static_cast<int>(required_double(values, "px"));
    config.camera.py = static_cast<int>(required_double(values, "py"));
    config.camera.fx = required_double(values, "fx");
    config.camera.fy = required_double(values, "fy");
    config.camera.k1 = required_double(values, "k1");
    config.camera.k2 = required_double(values, "k2");
    config.camera.cx = required_double(values, "cx");
    config.camera.cy = required_double(values, "cy");
    config.camera.dx = required_double(values, "dx");
    config.camera.dy = required_double(values, "dy");
    config.camera.pixel_error_x = required_double(values, "pixel_error_x");
    config.camera.pixel_error_y = required_double(values, "pixel_error_y");
    config.camera.angular_vision_x =
      required_double(values, "angular_vision_x");
    config.camera.angular_vision_y =
      required_double(values, "angular_vision_y");
    config.delta_t = required_double(values, "delta_t");
    return config;
  }

  std::map<int, Eigen::Vector3d> load_survey(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
      throw std::runtime_error("cannot open " + path);
    }
    std::map<int, Eigen::Vector3d> survey;
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
      std::stringstream row(line);
      std::string field;
      std::vector<double> values;
      while (std::getline(row, field, ',')) {
        if (!field.empty()) {
          values.push_back(std::stod(field));
        }
        if (values.size() == 4) {
          break;
        }
      }
      if (values.size() < 4) {
        continue;
      }
      survey[static_cast<int>(values[0])] =
        Eigen::Vector3d(values[1], values[2], values[3]);
    }
    return survey;
  }

  std::string with_slash(std::string directory) {
    if (!directory.empty() && directory.back() != '/') {
      directory.push_back('/');
    }
    return directory;
  }

  double trajectory_rmse(
    const Eigen::Matrix3Xd& estimated,
    const Eigen::Matrix3Xd& survey,
    const bool with_scale,
    double* scale
  ) {
    const Eigen::Matrix4d transform =
      Eigen::umeyama(estimated, survey, with_scale);
    if (scale != nullptr) {
      *scale = transform.block<3, 1>(0, 0).norm();
    }
    double squared = 0.0;
    for (int i = 0; i < estimated.cols(); ++i) {
      Eigen::Vector4d point;
      point << estimated.col(i), 1.0;
      const Eigen::Vector3d aligned = (transform * point).head<3>();
      squared += (aligned - survey.col(i)).squaredNorm();
    }
    return std::sqrt(squared / static_cast<double>(estimated.cols()));
  }

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "usage: agz_ate <image_dir> <config.yaml> <ground_truth.csv> "
                 "[end_index]\n";
    return 1;
  }

  const std::string image_dir = with_slash(argv[1]);
  const int end_index = argc > 4 ? std::stoi(argv[4]) : 350;
  const SlamConfig config = config_from_yaml(argv[2]);
  const std::map<int, Eigen::Vector3d> survey = load_survey(argv[3]);

  EKF ekf(config);
  std::map<int, Eigen::Vector3d> estimated;
  for (int id = 1; id <= end_index; ++id) {
    std::ostringstream filename;
    filename << image_dir << std::setw(5) << std::setfill('0') << id << ".jpg";
    const cv::Mat image = cv::imread(filename.str(), cv::IMREAD_COLOR);
    if (image.empty()) {
      std::cerr << "missing image " << filename.str() << "\n";
      return 1;
    }
    if (image.cols != config.camera.px || image.rows != config.camera.py) {
      std::cerr << "image " << id << " is " << image.cols << "x" << image.rows
                << ", config expects " << config.camera.px << "x"
                << config.camera.py << "\n";
      return 1;
    }
    ekf.process_frame(image);
    const Eigen::VectorXd state = ekf.state()->packed();
    const Eigen::MatrixXd& covariance = ekf.covariance_matrix()->matrix();
    if (!state.allFinite() || !covariance.allFinite()) {
      std::cerr << "non-finite state at image " << id << "\n";
      return 2;
    }
    estimated[id] = ekf.state()->position();
    if (id == 1 || id % 50 == 0) {
      std::cout << "frame " << id << "\n";
      std::cout.flush();
    }
  }

  std::vector<int> ids;
  for (const auto& [id, position] : survey) {
    (void)position;
    if (estimated.contains(id)) {
      ids.push_back(id);
    }
  }
  if (ids.size() < 3) {
    std::cerr << "need at least 3 paired poses, found " << ids.size() << "\n";
    return 1;
  }

  Eigen::Matrix3Xd filter_points(3, static_cast<int>(ids.size()));
  Eigen::Matrix3Xd survey_points(3, static_cast<int>(ids.size()));
  for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
    filter_points.col(i) = estimated.at(ids[i]);
    survey_points.col(i) = survey.at(ids[i]);
  }

  const Eigen::Matrix4d transform =
    Eigen::umeyama(filter_points, survey_points, true);
  const double scale = transform.block<3, 1>(0, 0).norm();
  double squared = 0.0;
  for (int i = 0; i < filter_points.cols(); ++i) {
    Eigen::Vector4d point;
    point << filter_points.col(i), 1.0;
    const Eigen::Vector3d aligned = (transform * point).head<3>();
    squared += (aligned - survey_points.col(i)).squaredNorm();
  }
  const double similarity =
    std::sqrt(squared / static_cast<double>(filter_points.cols()));
  const double rigid =
    trajectory_rmse(filter_points, survey_points, false, nullptr);

  {
    std::ofstream out("/tmp/agz_paths.csv");
    out << std::setprecision(12);
    out << "kind,id,x,y,z\n";
    for (int id = 1; id <= end_index; ++id) {
      Eigen::Vector4d point;
      point << estimated.at(id), 1.0;
      const Eigen::Vector3d aligned = (transform * point).head<3>();
      out << "filter," << id << "," << aligned.x() << "," << aligned.y() << ","
          << aligned.z() << "\n";
    }
    for (const int id : ids) {
      const Eigen::Vector3d& position = survey.at(id);
      out << "survey," << id << "," << position.x() << "," << position.y()
          << "," << position.z() << "\n";
    }
  }

  std::cout << std::setprecision(6);
  std::cout << "pairs " << ids.size() << "\n";
  std::cout << "scale " << scale << "\n";
  std::cout << "ate_similarity_m " << similarity << "\n";
  std::cout << "ate_rigid_m " << rigid << "\n";
  return 0;
}
