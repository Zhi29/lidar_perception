#include "read_json.hpp"

/**
namespace tools {
inline std::string ReadFileToString(const std::string &file_path) {
  std::ifstream fin(file_path);
  return std::string((std::istreambuf_iterator<char>(fin)),
                     std::istreambuf_iterator<char>());
}

inline nlohmann::json ReadFileToJson(const std::string &file_path) {
  std::string str = ReadFileToString(file_path);
  return nlohmann::json::parse(str);
}

template <typename Derived>
inline void JsonArrayToEigenMatrix(const nlohmann::json &json_array,
                                   Eigen::MatrixBase<Derived> *eigen_matrix_ptr) {
  CHECK(static_cast<int>(json_array.size()) == eigen_matrix_ptr->size())
    << json_array.size() << " " << eigen_matrix_ptr->size();
  // TODO: change to more effective implementation
  size_t rows = eigen_matrix_ptr->rows();
  size_t cols = eigen_matrix_ptr->cols();
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      (*eigen_matrix_ptr)(r, c) = json_array[r * cols + c];
    }
  }
  return;
}

} // namespace tools

struct LidarPerception {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarPerception(std::string const &perception_path)
        : LidarPerception(tools::ReadFileToJson(perception_path)) {}
    LidarPerception(nlohmann::json const &j){
        isKeyCube = j["children"]["data"]["isKeyCube"];
        isKeyPropertyCube = j["children"]["data"]["isKeyPropertyCube"];
        type = j["children"]["data"]["type"];

        height = j["children"]["height"];
        width = j["children"]["width"];
        length = j["children"]["length"];
        pitch = j["children"]["pitch"];
        roll = j["children"]["roll"];
        yaw = j["children"]["yaw"];
        x = j["children"]["x"];
        y = j["children"]["y"];
        z = j["children"]["z"];
        score = j["children"]["score"];
        tag = j["children"]["tag"];
        uuid = j["children"]["uuid"];







        bool isKeyCube;
        bool isKeyPropertyCube;
        std::string type;
        double height, length, width;
        double pitch, roll, yaw;
        double x, y, z;
        std::string tag, uuid;
        double score;

    }
};**/