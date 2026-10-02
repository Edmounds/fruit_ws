#ifndef FRUIT_PERCEPTION__RIPENESS_CLASSIFIER_HPP_
#define FRUIT_PERCEPTION__RIPENESS_CLASSIFIER_HPP_

#include <map>
#include <string>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

namespace fruit_perception
{

struct RipenessResult
{
  std::string maturity;
  float confidence;
};

// One camera has one in-flight frame. Requests and buffers are not thread-safe.
class RipenessClassifier
{
public:
  void initialize(ov::Core & core, const std::string & model_root, const std::string & engine);
  RipenessResult classify(const cv::Mat & rgb_roi, const std::string & crop);

private:
  std::string engine_;
  std::map<std::string, ov::InferRequest> requests_;
  cv::Mat resized_;
};

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__RIPENESS_CLASSIFIER_HPP_
