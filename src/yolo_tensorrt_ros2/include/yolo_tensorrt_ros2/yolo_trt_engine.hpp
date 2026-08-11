// Loi suy luan YOLOv8 tren TensorRT, dung chung boi node ROS chay realtime va
// cong cu auto_label chay offline. Chung phai la MOT doan code: nhan sinh ra
// tu tien xu ly khac voi luc chay that thi model hoc lech ma khong ai thay.
#ifndef YOLO_TENSORRT_ROS2__YOLO_TRT_ENGINE_HPP_
#define YOLO_TENSORRT_ROS2__YOLO_TRT_ENGINE_HPP_

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace yolo_trt
{

class CudaRuntimeError final : public std::runtime_error
{
public:
  explicit CudaRuntimeError(const std::string & message)
  : std::runtime_error(message) {}
};

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      const char * level = severity == Severity::kINTERNAL_ERROR ? "INTERNAL" :
        severity == Severity::kERROR ? "ERROR" : "WARN";
      fprintf(stderr, "[TensorRT %s] %s\n", level, message);
    }
  }
};

void check_cuda(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw CudaRuntimeError(
            std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

std::size_t tensor_volume(const nvinfer1::Dims & dims)
{
  std::size_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      throw std::runtime_error("Dynamic or invalid TensorRT tensor shape is unsupported");
    }
    volume *= static_cast<std::size_t>(dims.d[i]);
  }
  return volume;
}

std::string dims_to_string(const nvinfer1::Dims & dims)
{
  std::ostringstream out;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i != 0) {
      out << 'x';
    }
    out << dims.d[i];
  }
  return out.str();
}

float intersection_over_union(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float x1 = std::max(a.x, b.x);
  const float y1 = std::max(a.y, b.y);
  const float x2 = std::min(a.x + a.width, b.x + b.width);
  const float y2 = std::min(a.y + a.height, b.y + b.height);
  const float intersection =
    std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
  const float union_area = a.area() + b.area() - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

struct Detection
{
  cv::Rect2f box;
  float score{0.0F};
  int class_id{0};
};

struct LetterboxInfo
{
  float scale{1.0F};
  float pad_x{0.0F};
  float pad_y{0.0F};
};

class YoloTrtEngine
{
public:
  struct Options
  {
    std::string engine_path;
    int device_id{0};
    float confidence_threshold{0.30F};
    float nms_threshold{0.45F};
    int max_detections{300};
    // Rong = bo qua kiem tra. Neu co, so luong phai khop so class cua engine.
    std::vector<std::string> class_names;
  };

  explicit YoloTrtEngine(Options options)
  : options_(std::move(options))
  {
    load_engine();
  }

  ~YoloTrtEngine()
  {
    if (stream_ != nullptr) {
      cudaStreamSynchronize(stream_);
    }
    if (device_input_ != nullptr) {
      cudaFree(device_input_);
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  YoloTrtEngine(const YoloTrtEngine &) = delete;
  YoloTrtEngine & operator=(const YoloTrtEngine &) = delete;

  int input_width() const {return input_width_;}
  int input_height() const {return input_height_;}
  int class_count() const {return class_count_;}

  private:
  void load_engine()
  {
    std::ifstream file(options_.engine_path, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("Cannot open TensorRT engine: " + options_.engine_path);
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
      throw std::runtime_error("TensorRT engine is empty: " + options_.engine_path);
    }
    file.seekg(0, std::ios::beg);
    std::vector<char> serialized_engine(static_cast<std::size_t>(size));
    if (!file.read(serialized_engine.data(), size)) {
      throw std::runtime_error("Cannot read TensorRT engine: " + options_.engine_path);
    }

    check_cuda(cudaSetDevice(options_.device_id), "cudaSetDevice");
    runtime_.reset(nvinfer1::createInferRuntime(tensorrt_logger_));
    if (!runtime_) {
      throw std::runtime_error("TensorRT createInferRuntime failed");
    }
    engine_.reset(
      runtime_->deserializeCudaEngine(
        serialized_engine.data(), serialized_engine.size()));
    if (!engine_) {
      throw std::runtime_error(
              "TensorRT cannot deserialize engine; verify JetPack/TensorRT compatibility");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("TensorRT createExecutionContext failed");
    }

    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
      } else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT) {
        output_name_ = name;
      }
    }
    if (input_name_.empty() || output_name_.empty() || engine_->getNbIOTensors() != 2) {
      throw std::runtime_error(
              "Expected one input and one output tensor in YOLO engine");
    }
    if (engine_->getTensorDataType(input_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
      engine_->getTensorDataType(output_name_.c_str()) != nvinfer1::DataType::kFLOAT)
    {
      throw std::runtime_error("Only FP32 engine I/O tensors are supported");
    }

    const nvinfer1::Dims input_dims = engine_->getTensorShape(input_name_.c_str());
    const nvinfer1::Dims output_dims = engine_->getTensorShape(output_name_.c_str());
    if (input_dims.nbDims != 4 || input_dims.d[0] != 1 || input_dims.d[1] != 3) {
      throw std::runtime_error(
              "Expected YOLO input shape 1x3xHxW, got " + dims_to_string(input_dims));
    }
    if (output_dims.nbDims != 3 || output_dims.d[0] != 1 || output_dims.d[1] <= 4) {
      throw std::runtime_error(
              "Expected YOLOv8 output shape 1x(4+classes)xN, got " +
              dims_to_string(output_dims));
    }

    input_height_ = input_dims.d[2];
    input_width_ = input_dims.d[3];
    output_channels_ = output_dims.d[1];
    output_candidates_ = output_dims.d[2];
    class_count_ = output_channels_ - 4;

    if (!options_.class_names.empty() &&
      static_cast<int>(options_.class_names.size()) != class_count_)
    {
      throw std::runtime_error(
              "class_names contains " + std::to_string(options_.class_names.size()) +
              " entries but the engine has " + std::to_string(class_count_) + " classes");
    }

    input_host_.resize(tensor_volume(input_dims));
    output_host_.resize(tensor_volume(output_dims));
    check_cuda(
      cudaMalloc(&device_input_, input_host_.size() * sizeof(float)),
      "cudaMalloc(input)");
    check_cuda(
      cudaMalloc(&device_output_, output_host_.size() * sizeof(float)),
      "cudaMalloc(output)");
    check_cuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

    if (!context_->setTensorAddress(input_name_.c_str(), device_input_) ||
      !context_->setTensorAddress(output_name_.c_str(), device_output_))
    {
      throw std::runtime_error("TensorRT setTensorAddress failed");
    }
  }
  LetterboxInfo preprocess(const cv::Mat & source)
  {
    const float scale = std::min(
      static_cast<float>(input_width_) / static_cast<float>(source.cols),
      static_cast<float>(input_height_) / static_cast<float>(source.rows));
    const int resized_width =
      std::max(1, static_cast<int>(std::round(source.cols * scale)));
    const int resized_height =
      std::max(1, static_cast<int>(std::round(source.rows * scale)));
    const int pad_left = (input_width_ - resized_width) / 2;
    const int pad_top = (input_height_ - resized_height) / 2;

    cv::Mat resized;
    cv::resize(
      source, resized, cv::Size(resized_width, resized_height),
      0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat letterboxed(
      input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(
      letterboxed(cv::Rect(pad_left, pad_top, resized_width, resized_height)));

    cv::Mat blob = cv::dnn::blobFromImage(
      letterboxed, 1.0 / 255.0, cv::Size(input_width_, input_height_),
      cv::Scalar(), true, false, CV_32F);
    if (!blob.isContinuous() || blob.total() != input_host_.size()) {
      throw std::runtime_error("OpenCV produced an invalid input blob");
    }
    std::copy(
      blob.ptr<float>(), blob.ptr<float>() + blob.total(), input_host_.begin());

    return LetterboxInfo{
      scale, static_cast<float>(pad_left), static_cast<float>(pad_top)};
  }

  public:
  std::vector<Detection> infer(const cv::Mat & source)
  {
    const LetterboxInfo letterbox = preprocess(source);

    check_cuda(
      cudaMemcpyAsync(
        device_input_, input_host_.data(), input_host_.size() * sizeof(float),
        cudaMemcpyHostToDevice, stream_),
      "cudaMemcpyAsync(H2D)");
    if (!context_->enqueueV3(stream_)) {
      throw CudaRuntimeError("TensorRT enqueueV3 failed");
    }
    check_cuda(
      cudaMemcpyAsync(
        output_host_.data(), device_output_, output_host_.size() * sizeof(float),
        cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync(D2H)");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

    std::vector<Detection> candidates;
    candidates.reserve(256);
    for (int index = 0; index < output_candidates_; ++index) {
      int best_class = 0;
      float best_score = -std::numeric_limits<float>::infinity();
      for (int class_id = 0; class_id < class_count_; ++class_id) {
        const float score =
          output_host_[static_cast<std::size_t>(4 + class_id) *
          output_candidates_ + index];
        if (score > best_score) {
          best_score = score;
          best_class = class_id;
        }
      }
      if (!std::isfinite(best_score) || best_score < options_.confidence_threshold) {
        continue;
      }

      const float center_x = output_host_[index];
      const float center_y = output_host_[output_candidates_ + index];
      const float width = output_host_[2 * output_candidates_ + index];
      const float height = output_host_[3 * output_candidates_ + index];
      if (!(width > 0.0F && height > 0.0F)) {
        continue;
      }
      candidates.push_back(
        Detection{
          cv::Rect2f(
            center_x - width * 0.5F, center_y - height * 0.5F, width, height),
          best_score,
          best_class});
    }

    // NMS must be class-aware. A tray box deliberately contains its cartridge
    // boxes, so class-agnostic NMS would erase valid cartridge detections.
    std::vector<Detection> kept;
    kept.reserve(candidates.size());
    for (int class_id = 0; class_id < class_count_; ++class_id) {
      std::vector<std::size_t> order;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].class_id == class_id) {
          order.push_back(i);
        }
      }
      std::sort(
        order.begin(), order.end(),
        [&candidates](std::size_t left, std::size_t right) {
          return candidates[left].score > candidates[right].score;
        });

      std::vector<bool> suppressed(order.size(), false);
      for (std::size_t i = 0; i < order.size(); ++i) {
        if (suppressed[i]) {
          continue;
        }
        kept.push_back(candidates[order[i]]);
        if (static_cast<int>(kept.size()) >= options_.max_detections) {
          break;
        }
        for (std::size_t j = i + 1; j < order.size(); ++j) {
          if (!suppressed[j] &&
            intersection_over_union(
              candidates[order[i]].box, candidates[order[j]].box) >
            options_.nms_threshold)
          {
            suppressed[j] = true;
          }
        }
      }
      if (static_cast<int>(kept.size()) >= options_.max_detections) {
        break;
      }
    }

    for (auto & detection : kept) {
      float x1 = (detection.box.x - letterbox.pad_x) / letterbox.scale;
      float y1 = (detection.box.y - letterbox.pad_y) / letterbox.scale;
      float x2 =
        (detection.box.x + detection.box.width - letterbox.pad_x) /
        letterbox.scale;
      float y2 =
        (detection.box.y + detection.box.height - letterbox.pad_y) /
        letterbox.scale;
      x1 = std::clamp(x1, 0.0F, static_cast<float>(source.cols));
      y1 = std::clamp(y1, 0.0F, static_cast<float>(source.rows));
      x2 = std::clamp(x2, 0.0F, static_cast<float>(source.cols));
      y2 = std::clamp(y2, 0.0F, static_cast<float>(source.rows));
      detection.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
    }
    kept.erase(
      std::remove_if(
        kept.begin(), kept.end(),
        [](const Detection & detection) {
          return detection.box.width < 1.0F || detection.box.height < 1.0F;
        }),
      kept.end());
    return kept;
  }

private:
  Options options_;
  TensorRtLogger tensorrt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_{nullptr};
  void * device_input_{nullptr};
  void * device_output_{nullptr};
  std::vector<float> input_host_;
  std::vector<float> output_host_;
  std::string input_name_;
  std::string output_name_;
  int input_width_{0};
  int input_height_{0};
  int output_channels_{0};
  int output_candidates_{0};
  int class_count_{0};
};

}  // namespace yolo_trt

#endif  // YOLO_TENSORRT_ROS2__YOLO_TRT_ENGINE_HPP_
