#include <chrono>
#include <cerrno>
#include <mutex>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

extern "C" {
#include <modbus/modbus.h>
}

using namespace std::chrono_literals;

// =========================================================
//  RS485BusNode  —  RevPi A
//  ATV320 VFD control over Modbus RTU
//
//  Non-blocking connection rule:
//   - Modbus offline → init OK, vẫn nhận subscription
//   - cmd_run đến lúc Modbus offline → lưu desired_run_, không mất lệnh
//   - Reconnect → tự arm VFD + apply desired_run_ + desired_freq_ tự động
//
//  PUBLISH:
//    /vfd/freq_fb   Float32  Tần số phản hồi từ VFD (Hz), 5 Hz
//    /vfd/state     Int32    0=disabled 1=enabled 2=running
//    /vfd/status    String   "OK" | "ERROR" (connection)
//
//  SUBSCRIBE:
//    /vfd/cmd_run   Bool     true=RUN; false=STOP
//    /vfd/cmd_freq  Float32  Tần số đích (Hz), clamp [-50, 50] (âm = ngược)
// =========================================================

class RS485BusNode : public rclcpp::Node
{
public:
  RS485BusNode()
  : Node("rs485_bus")
  {
    // ── VFD registers + commands (ATV320 DSP 402) ─────────
    REG_CMD_     = 8501;  REG_LFR_     = 8502;
    CMD_DISABLE_ = 0x0000; CMD_ENABLE_ = 0x0006; CMD_RUN_FWD_ = 0x000F;

    // ── Defaults baked vào node (start script KHÔNG set --ros-args) ──
    // ATV320 ở RevPi A, đấu ngược chiều băng tải → ref_hz mặc định -50.0.
    slave_id_  = declare_parameter<int>("slave_id",    2);
    ref_hz_    = declare_parameter<double>("ref_hz",  -50.0);
    ref_scale_ = declare_parameter<double>("ref_scale",10.0);

    // ── RS485 / Modbus RTU ───────────────────────────────
    port_         = declare_parameter<std::string>("port",       "/dev/ttyRS485");
    baudrate_     = declare_parameter<int>("baudrate",            9600);
    parity_str_   = declare_parameter<std::string>("parity",     "N");
    stopbits_     = declare_parameter<int>("stopbits",            1);
    bytesize_     = declare_parameter<int>("bytesize",            8);
    timeout_s_    = declare_parameter<double>("timeout",          1.0);
    double turnaround_ms = declare_parameter<double>("turnaround_ms", 5.0);
    turnaround_   = turnaround_ms / 1000.0;

    // Desired state — nguồn sự thật từ topic, áp dụng dần khi Modbus sẵn sàng.
    desired_run_  = false;
    desired_freq_ = ref_hz_;
    actual_running_ = false;
    vfd_armed_    = false;

    // Init non-blocking — không crash khi serial port chưa có.
    try_open_modbus();  // log lỗi, không return — poll sẽ retry.

    // ── Publishers ───────────────────────────────────────
    pub_freq_      = create_publisher<std_msgs::msg::Float32>("/vfd/freq_fb", 10);
    pub_vfd_state_ = create_publisher<std_msgs::msg::Int32>  ("/vfd/state",   10);
    pub_status_    = create_publisher<std_msgs::msg::String> ("/vfd/status",  10);

    // ── Safety timeout: VFD chạy quá run_timeout_s_ giây mà chưa thấy STOP
    //    (S3 trigger từ Pi 5) → tự dừng. Bảo vệ trường hợp S3 nhiễu/cảm biến
    //    chết/vfd_logic_node treo. Reset timer mỗi khi cmd_run flip false→true.
    run_timeout_s_ = declare_parameter<double>("run_timeout_s", 30.0);
    run_start_steady_ = std::chrono::steady_clock::time_point{};  // not running

    // ── Subscribers — chỉ lưu desired, apply trong reconcile ─
    sub_vfd_run_ = create_subscription<std_msgs::msg::Bool>(
      "/vfd/cmd_run", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        bool prev = desired_run_.exchange(msg->data);
        if (prev != msg->data) {
          RCLCPP_INFO(get_logger(), "[VFD] /vfd/cmd_run → %s (desired)",
                      msg->data ? "RUN" : "STOP");
          // Edge → reset / clear safety timer. Sau timeout desired_run_ đã
          // set false; heartbeat re-publish true sẽ trigger edge này lại →
          // VFD chạy thêm 50s nữa (user OK với behavior này).
          std::lock_guard<std::mutex> lk(run_timer_mutex_);
          run_start_steady_ = msg->data
              ? std::chrono::steady_clock::now()
              : std::chrono::steady_clock::time_point{};
        }
        reconcile();
      });

    sub_vfd_freq_ = create_subscription<std_msgs::msg::Float32>(
      "/vfd/cmd_freq", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        float f = msg->data;
        if (f < -50.0f) f = -50.0f;
        if (f >  50.0f) f =  50.0f;
        desired_freq_ = f;
        RCLCPP_INFO(get_logger(), "[VFD] /vfd/cmd_freq → %.1f Hz (desired, %s)",
                    f, f < 0.0f ? "NGƯỢC" : "THUẬN");
        reconcile();
      });

    // Poll trạng thái VFD + reconnect + reconcile.
    timer_freq_ = create_wall_timer(200ms,
      std::bind(&RS485BusNode::poll_loop, this));

    RCLCPP_INFO(get_logger(),
      "[VFD] ATV320 node ready  slave_id=%d  ref_hz=%.1f  ref_scale=%.1f  port=%s",
      slave_id_, ref_hz_, ref_scale_, port_.c_str());
  }

  ~RS485BusNode()
  {
    if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); }
  }

private:

  // ── Modbus connect — non-blocking, không exit khi fail. ──
  bool try_open_modbus()
  {
    char parity = parity_str_.empty() ? 'N' : static_cast<char>(parity_str_[0]);
    ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, parity, bytesize_, stopbits_);
    if (!ctx_) {
      RCLCPP_ERROR(get_logger(), "[RS485] modbus_new_rtu failed (will retry)");
      return false;
    }
    modbus_set_response_timeout(ctx_,
      static_cast<uint32_t>(timeout_s_),
      static_cast<uint32_t>((timeout_s_ - static_cast<int>(timeout_s_)) * 1e6));
    if (modbus_connect(ctx_) == -1) {
      RCLCPP_WARN(get_logger(), "[RS485] Cannot connect (will retry): %s",
                  modbus_strerror(errno));
      modbus_free(ctx_);
      ctx_ = nullptr;
      connection_ok_ = false;
      vfd_armed_ = false;
      return false;
    }
    connection_ok_ = true;
    RCLCPP_INFO(get_logger(), "[RS485] Connected  port=%s  baud=%d",
                port_.c_str(), baudrate_);
    return true;
  }

  // ── Arm VFD (reset fault + ENABLE) — gọi sau mỗi reconnect. ──
  bool arm_vfd()
  {
    if (!connection_ok_) return false;
    bool ok = true;
    ok &= write_reg(REG_CMD_, 0x0080, slave_id_);              // reset fault
    rclcpp::sleep_for(100ms);
    ok &= write_reg(REG_CMD_, CMD_DISABLE_, slave_id_);
    rclcpp::sleep_for(100ms);
    ok &= write_reg(REG_LFR_, static_cast<int>(desired_freq_ * ref_scale_), slave_id_);
    ok &= write_reg(REG_CMD_, CMD_ENABLE_, slave_id_);
    if (ok) {
      vfd_armed_ = true;
      actual_running_ = false;
      RCLCPP_INFO(get_logger(), "[VFD] Armed (reset fault → ENABLE)");
    } else {
      RCLCPP_WARN(get_logger(), "[VFD] arm_vfd: 1+ write failed");
    }
    return ok;
  }

  bool write_reg(int reg, int val, int sid)
  {
    if (!ctx_) return false;
    std::lock_guard<std::mutex> lk(bus_mutex_);
    modbus_set_slave(ctx_, sid);
    if (modbus_write_register(ctx_, reg, val) == -1) {
      RCLCPP_WARN(get_logger(), "[Write] reg=%d val=0x%04X err=%s",
                  reg, val, modbus_strerror(errno));
      return false;
    }
    sleep_ta();
    return true;
  }

  bool read_reg(int reg, int sid, int &out)
  {
    if (!ctx_) return false;
    std::lock_guard<std::mutex> lk(bus_mutex_);
    modbus_set_slave(ctx_, sid);
    uint16_t r = 0;
    if (modbus_read_registers(ctx_, reg, 1, &r) == -1) return false;
    out = static_cast<int16_t>(r);  // signed: âm = chiều ngược
    sleep_ta();
    return true;
  }

  void sleep_ta()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(turnaround_ * 1000)));
  }

  void pub_int(rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr &p, int v)
  { std_msgs::msg::Int32 m; m.data = v; p->publish(m); }

  void pub_float(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &p, float v)
  { std_msgs::msg::Float32 m; m.data = v; p->publish(m); }

  void pub_str(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &p, const std::string &s)
  { std_msgs::msg::String m; m.data = s; p->publish(m); }

  // ═════════════════════════════════════════════════════════════════
  //  ⚠️  CRITICAL ZONE — đọc memory feedback_critical_code_zones.md trước khi sửa.
  //  INVARIANT: subscription callback CHỈ lưu desired_* atomic, KHÔNG gọi
  //  write_reg trực tiếp. reconcile() idempotent. arm_vfd() sau reconnect.
  //  Đừng dùng pattern cũ: `if msg->data && !running_` + `running_ = msg->data`
  //  sau write — sẽ MẤT LỆNH nếu Modbus chưa connect lúc cmd_run đến (running_
  //  set true nhưng write fail → heartbeat sau đó skip vì check !running_).
  // ═════════════════════════════════════════════════════════════════
  // Reconcile: drive VFD theo desired_* khi đã arm.
  // Idempotent — gọi nhiều lần OK; chỉ ghi khi state lệch.
  void reconcile()
  {
    if (!connection_ok_ || !vfd_armed_) return;

    bool want_run = desired_run_.load();
    float want_freq = desired_freq_.load();

    // Sync freq trước (VFD có thể đổi freq khi đang chạy).
    int want_raw = static_cast<int>(want_freq * ref_scale_);
    if (want_raw != last_freq_written_) {
      if (write_reg(REG_LFR_, want_raw, slave_id_)) {
        last_freq_written_ = want_raw;
        ref_hz_ = want_freq;
      }
    }

    if (want_run && !actual_running_) {
      // Start sequence: Switch On → Enable Operation
      write_reg(REG_LFR_, want_raw, slave_id_);  // force freq lần nữa
      if (write_reg(REG_CMD_, 0x0007, slave_id_)) {
        rclcpp::sleep_for(50ms);
        if (write_reg(REG_CMD_, CMD_RUN_FWD_, slave_id_)) {
          actual_running_ = true;
          RCLCPP_INFO(get_logger(), "[VFD] RUN @ %.1f Hz", want_freq);
        }
      }
    } else if (!want_run && actual_running_) {
      // Stop: về Ready-to-switch-on (giữ DC-bus charge).
      if (write_reg(REG_CMD_, CMD_ENABLE_, slave_id_)) {
        actual_running_ = false;
        RCLCPP_INFO(get_logger(), "[VFD] STOP");
      }
    }
  }

  // Safety auto-stop: nếu desired_run_ giữ true quá run_timeout_s_ giây mà
  // chưa có /vfd/cmd_run=false từ Pi 5 (S3 nhiễu/cảm biến chết/vfd_logic treo)
  // → tự stop, không cảnh báo. Reset bằng cách publish lại cmd_run=true sau STOP.
  void check_run_timeout()
  {
    std::chrono::steady_clock::time_point start;
    {
      std::lock_guard<std::mutex> lk(run_timer_mutex_);
      start = run_start_steady_;
    }
    if (start == std::chrono::steady_clock::time_point{}) return;  // not running
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    if (elapsed < run_timeout_s_) return;

    // Timeout — force stop. desired_run_=false, clear timer. reconcile()
    // ghi REG_CMD=ENABLE (stop) ngay tick. Nếu vfd_logic heartbeat publish
    // cmd_run=true lại (S1/S2 vẫn ON) → sub callback bắt edge → reset timer
    // → VFD chạy thêm 50s nữa (user accept behavior).
    desired_run_ = false;
    {
      std::lock_guard<std::mutex> lk(run_timer_mutex_);
      run_start_steady_ = std::chrono::steady_clock::time_point{};
    }
    RCLCPP_INFO(get_logger(),
        "[VFD] Safety timeout %.1fs — auto STOP (no STOP signal from Pi 5)",
        run_timeout_s_);
  }

  void poll_loop()
  {
    // ── Bước 0: kiểm tra safety run timeout (independent với Modbus state) ──
    check_run_timeout();

    // ── Bước 1: reconnect khi Modbus down ──
    if (!connection_ok_) {
      if (++reconnect_count_ >= 10) {  // ~2s
        reconnect_count_ = 0;
        std::lock_guard<std::mutex> lk(bus_mutex_);
        if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); ctx_ = nullptr; }
        if (try_open_modbus()) {
          pub_str(pub_status_, "OK");
        } else {
          pub_str(pub_status_, "ERROR");
        }
      }
      return;
    }

    // ── Bước 2: vừa connect xong → arm + reconcile desired. ──
    if (!vfd_armed_) {
      if (arm_vfd()) {
        last_freq_written_ = INT_MIN;  // force re-write freq
        reconcile();                   // apply desired_run_ lập tức
      }
      return;
    }

    // ── Bước 3: poll freq feedback ──
    int raw = 0;
    if (!read_reg(REG_LFR_, slave_id_, raw)) {
      if (++fail_count_ >= 5) {
        RCLCPP_WARN(get_logger(), "[VFD] Read fail 5x → reconnect");
        connection_ok_ = false;
        vfd_armed_ = false;
        fail_count_ = 0;
        pub_str(pub_status_, "ERROR");
      }
      return;
    }
    fail_count_ = 0;
    pub_float(pub_freq_, raw / 10.0f);
    pub_int(pub_vfd_state_, actual_running_ ? 2 : 1);

    // ── Bước 4: reconcile mỗi tick (phòng desired đổi giữa các sub callback) ──
    reconcile();
  }

  // ── Members ──────────────────────────────────────────────

  modbus_t   *ctx_{nullptr};
  std::mutex  bus_mutex_;
  std::atomic<bool> connection_ok_{false};
  std::atomic<bool> vfd_armed_{false};
  std::atomic<bool> actual_running_{false};

  // Desired state — nguồn từ topic, atomic vì cross-thread (sub callback + timer).
  std::atomic<bool>  desired_run_{false};
  std::atomic<float> desired_freq_{0.0f};
  int last_freq_written_{INT_MIN};

  // Safety auto-stop timer: track thời điểm cmd_run flip false→true, force
  // stop nếu vượt run_timeout_s_ (default 30s). Bảo vệ khi S3 nhiễu/cảm biến
  // chết. Sau timeout VFD dừng; nếu vfd_logic heartbeat re-publish true (S1/S2
  // vẫn ON) → bắt edge → chạy thêm một chu kỳ timeout nữa. Không latch.
  std::mutex run_timer_mutex_;
  std::chrono::steady_clock::time_point run_start_steady_{};
  double run_timeout_s_{30.0};

  int reconnect_count_{0};
  int fail_count_{0};

  // RS485 config
  std::string port_;
  int         baudrate_;
  std::string parity_str_;
  int         stopbits_;
  int         bytesize_;
  double      timeout_s_;
  double      turnaround_;

  // VFD registers/commands
  int    REG_CMD_, REG_LFR_;
  int    CMD_DISABLE_, CMD_ENABLE_, CMD_RUN_FWD_;
  int    slave_id_;
  double ref_hz_, ref_scale_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_freq_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   pub_vfd_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_status_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    sub_vfd_run_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vfd_freq_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_freq_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RS485BusNode>());
  rclcpp::shutdown();
  return 0;
}
