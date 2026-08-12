// Sinh nhan YOLO tu dong cho anh dataset bang chinh engine dang chay production.
//
// Dung de mo bai: chay engine cu len anh moi thu duoc, lay ra file .txt, roi mo
// labelImg sua tay. Sua nhan nhanh hon ve tu dau rat nhieu.
//
// Vi sao khong dung ultralytics/onnxruntime cho tien: may nay khong co san
// (khong co torch, khong co onnxruntime, OpenCV 4.5.4 khong doc noi YOLOv8
// ONNX). Va quan trong hon, nhan phai sinh ra tu DUNG doan tien/hau xu ly ma
// node chay that dung — yolo_trt_engine.hpp la mot ban duy nhat cho ca hai.
//
//   ros2 run yolo_tensorrt_ros2 auto_label
//       --engine /home/nhan/models/data_input_hp_final_2_fp16.engine
//       --images ~/Datasets/.../input/images
//
// Nhan xuat ra dung dinh dang YOLO: "<class> <cx> <cy> <w> <h>", toa do chuan
// hoa 0..1 theo kich thuoc anh (640x360), moi dong mot vat.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "yolo_tensorrt_ros2/yolo_trt_engine.hpp"

namespace fs = std::filesystem;

namespace
{

struct Args
{
  std::string engine_path;
  std::string images_dir;
  std::string labels_dir;      // rong = <images_dir>/../labels
  float confidence{0.25F};
  float nms{0.45F};
  int max_detections{300};
  bool overwrite{false};
  bool dry_run{false};
  int limit{0};                // 0 = tat ca
};

void print_usage()
{
  std::cout <<
    "auto_label — sinh nhan YOLO .txt tu TensorRT engine\n\n"
    "  --engine PATH     engine .engine (bat buoc)\n"
    "  --images DIR      thu muc anh (bat buoc)\n"
    "  --labels DIR      thu muc nhan (mac dinh <images>/../labels)\n"
    "  --conf F          nguong tin cay, mac dinh 0.25\n"
    "                    (thap hon production 0.60 co chu y: them box thua de\n"
    "                     nguoi xoa de hon la tim box thieu de them)\n"
    "  --nms F           nguong NMS, mac dinh 0.45\n"
    "  --max-det N       so box toi da moi anh, mac dinh 300\n"
    "  --overwrite       ghi de nhan da co (mac dinh: BO QUA, giu nhan sua tay)\n"
    "  --limit N         chi lam N anh dau, de thu truoc\n"
    "  --dry-run         chi dem, khong ghi file\n";
}

bool parse_args(int argc, char ** argv, Args & args)
{
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    auto next = [&](const char * name) -> std::string {
        if (i + 1 >= argc) {
          std::cerr << "Thieu gia tri cho " << name << "\n";
          std::exit(2);
        }
        return argv[++i];
      };
    if (flag == "--engine") {args.engine_path = next("--engine");} else if (flag == "--images") {
      args.images_dir = next("--images");
    } else if (flag == "--labels") {args.labels_dir = next("--labels");} else if (flag == "--conf") {
      args.confidence = std::stof(next("--conf"));
    } else if (flag == "--nms") {args.nms = std::stof(next("--nms"));} else if (flag == "--max-det") {
      args.max_detections = std::stoi(next("--max-det"));
    } else if (flag == "--limit") {args.limit = std::stoi(next("--limit"));} else if (flag ==
      "--overwrite")
    {
      args.overwrite = true;
    } else if (flag == "--dry-run") {args.dry_run = true;} else if (flag == "-h" ||
      flag == "--help")
    {
      print_usage();
      std::exit(0);
    } else {
      std::cerr << "Tham so la: " << flag << "\n\n";
      print_usage();
      return false;
    }
  }
  if (args.engine_path.empty() || args.images_dir.empty()) {
    print_usage();
    return false;
  }
  return true;
}

bool is_image(const fs::path & p)
{
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp";
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!parse_args(argc, argv, args)) {
    return 2;
  }

  const fs::path images_dir(args.images_dir);
  if (!fs::is_directory(images_dir)) {
    std::cerr << "Khong phai thu muc: " << images_dir << "\n";
    return 1;
  }
  const fs::path labels_dir = args.labels_dir.empty()
    ? images_dir.parent_path() / "labels"
    : fs::path(args.labels_dir);

  std::vector<fs::path> images;
  for (const auto & entry : fs::directory_iterator(images_dir)) {
    if (entry.is_regular_file() && is_image(entry.path())) {
      images.push_back(entry.path());
    }
  }
  std::sort(images.begin(), images.end());
  if (images.empty()) {
    std::cerr << "Khong thay anh nao trong " << images_dir << "\n";
    return 1;
  }
  if (args.limit > 0 && static_cast<int>(images.size()) > args.limit) {
    images.resize(static_cast<std::size_t>(args.limit));
  }

  std::unique_ptr<yolo_trt::YoloTrtEngine> engine;
  try {
    engine = std::make_unique<yolo_trt::YoloTrtEngine>(
      yolo_trt::YoloTrtEngine::Options{
        args.engine_path, 0, args.confidence, args.nms, args.max_detections, {}});
  } catch (const std::exception & error) {
    std::cerr << "Khong nap duoc engine: " << error.what() << "\n";
    return 1;
  }

  std::cout << "engine   : " << args.engine_path << "\n"
            << "input    : " << engine->input_width() << "x" << engine->input_height()
            << ", " << engine->class_count() << " class\n"
            << "anh      : " << images.size() << " file trong " << images_dir << "\n"
            << "nhan     : " << labels_dir << (args.dry_run ? "  (DRY RUN)" : "") << "\n"
            << "conf     : " << args.confidence << "   nms: " << args.nms << "\n\n";

  if (!args.dry_run) {
    fs::create_directories(labels_dir);
  }

  std::map<int, long> per_class;
  long total_boxes = 0;
  int written = 0, skipped_existing = 0, failed = 0;
  std::vector<std::string> empty_images;

  for (std::size_t i = 0; i < images.size(); ++i) {
    const fs::path & image_path = images[i];
    const fs::path label_path = labels_dir / (image_path.stem().string() + ".txt");

    if (!args.overwrite && fs::exists(label_path)) {
      ++skipped_existing;
      continue;
    }

    const cv::Mat frame = cv::imread(image_path.string(), cv::IMREAD_COLOR);
    if (frame.empty()) {
      std::cerr << "  [LOI] khong doc duoc " << image_path.filename() << "\n";
      ++failed;
      continue;
    }

    std::vector<yolo_trt::Detection> detections;
    try {
      detections = engine->infer(frame);
    } catch (const std::exception & error) {
      std::cerr << "  [LOI] " << image_path.filename() << ": " << error.what() << "\n";
      ++failed;
      continue;
    }

    // YOLO txt: toa do chuan hoa theo kich thuoc anh goc (640x360), KHONG phai
    // theo khung 640x640 da letterbox. infer() da tra box ve he anh goc roi.
    const float iw = static_cast<float>(frame.cols);
    const float ih = static_cast<float>(frame.rows);
    std::ostringstream body;
    body << std::fixed << std::setprecision(6);
    int kept = 0;
    for (const auto & d : detections) {
      const float cx = (d.box.x + d.box.width * 0.5F) / iw;
      const float cy = (d.box.y + d.box.height * 0.5F) / ih;
      const float w = d.box.width / iw;
      const float h = d.box.height / ih;
      if (w <= 0.0F || h <= 0.0F) {continue;}
      body << d.class_id << ' '
           << std::clamp(cx, 0.0F, 1.0F) << ' ' << std::clamp(cy, 0.0F, 1.0F) << ' '
           << std::clamp(w, 0.0F, 1.0F) << ' ' << std::clamp(h, 0.0F, 1.0F) << '\n';
      ++per_class[d.class_id];
      ++kept;
    }
    total_boxes += kept;
    if (kept == 0) {
      empty_images.push_back(image_path.filename().string());
    }

    if (!args.dry_run) {
      // Anh khong co vat nao van phai co file .txt RONG — do la hard negative
      // hop le voi Ultralytics. Thieu file thi anh bi coi la chua gan nhan.
      std::ofstream out(label_path);
      if (!out) {
        std::cerr << "  [LOI] khong ghi duoc " << label_path << "\n";
        ++failed;
        continue;
      }
      out << body.str();
    }
    ++written;

    if ((i + 1) % 50 == 0 || i + 1 == images.size()) {
      std::cout << "  " << (i + 1) << "/" << images.size()
                << "  box=" << total_boxes << std::endl;
    }
  }

  std::cout << "\n--- tong ket ---\n"
            << "da ghi nhan      : " << written << "\n"
            << "bo qua (da co)   : " << skipped_existing << "\n"
            << "loi              : " << failed << "\n"
            << "tong box         : " << total_boxes << "\n";
  for (const auto & [class_id, count] : per_class) {
    std::cout << "  class " << class_id << " : " << count << "\n";
  }

  if (!empty_images.empty()) {
    std::cout << "\n" << empty_images.size()
              << " anh KHONG co box nao — kiem tra tay truoc tien, day thuong la\n"
                 "cho model cu bo sot hoac anh that su rong:\n";
    for (std::size_t i = 0; i < empty_images.size() && i < 20; ++i) {
      std::cout << "  " << empty_images[i] << "\n";
    }
    if (empty_images.size() > 20) {
      std::cout << "  ... con " << (empty_images.size() - 20) << " anh nua\n";
    }
  }

  std::cout << "\nNhan la DE XUAT cua model cu, khong phai su that. Mo labelImg\n"
               "sua lai truoc khi train.\n";
  return failed > 0 ? 1 : 0;
}
