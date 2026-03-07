/**
 * @file src/platform/linux/input/inputtino_esp32.cpp
 * @brief Definitions for ESP32 serial controller transport.
 */

// standard includes
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstring>

// platform includes
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// lib includes
#include <nlohmann/json.hpp>

// local includes
#include "inputtino_esp32.h"
#include "src/logging.h"
#include "src/platform/common.h"

using namespace std::literals;

namespace platf::esp32 {
  namespace {
    speed_t baud_to_constant(int baud) {
      switch (baud) {
        case 9600:
          return B9600;
        case 19200:
          return B19200;
        case 38400:
          return B38400;
        case 57600:
          return B57600;
        case 115200:
          return B115200;
#ifdef B230400
        case 230400:
          return B230400;
#endif
#ifdef B460800
        case 460800:
          return B460800;
#endif
#ifdef B500000
        case 500000:
          return B500000;
#endif
#ifdef B576000
        case 576000:
          return B576000;
#endif
#ifdef B921600
        case 921600:
          return B921600;
#endif
#ifdef B1000000
        case 1000000:
          return B1000000;
#endif
#ifdef B1152000
        case 1152000:
          return B1152000;
#endif
#ifdef B1500000
        case 1500000:
          return B1500000;
#endif
#ifdef B2000000
        case 2000000:
          return B2000000;
#endif
#ifdef B2500000
        case 2500000:
          return B2500000;
#endif
#ifdef B3000000
        case 3000000:
          return B3000000;
#endif
#ifdef B3500000
        case 3500000:
          return B3500000;
#endif
#ifdef B4000000
        case 4000000:
          return B4000000;
#endif
        default:
          return B115200;
      }
    }

    std::string to_json_line(const nlohmann::json &payload) {
      return payload.dump() + "\n";
    }

    std::string make_stick_line(std::string_view stick_id, std::int16_t x, std::int16_t y) {
      nlohmann::json payload {
        {"action", "stick"},
        {"stick_id", std::string(stick_id)},
        {"x", x},
        {"y", y},
      };
      return to_json_line(payload);
    }

    std::string make_hat_line(std::string_view direction) {
      nlohmann::json payload {
        {"action", "hat"},
        {"direction", std::string(direction)},
      };
      return to_json_line(payload);
    }

    std::string normalized_input_policy(std::string policy) {
      std::transform(policy.begin(), policy.end(), policy.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });

      if (policy == "bluetooth"sv || policy == "bt"sv) {
        return "websocket"s;
      }
      if (policy == "wifi"sv) {
        return "http"s;
      }

      return policy;
    }

    bool write_all(int fd, const char *data, std::size_t size) {
      std::size_t sent = 0;
      while (sent < size) {
        const auto rc = ::write(fd, data + sent, size - sent);
        if (rc > 0) {
          sent += static_cast<std::size_t>(rc);
          continue;
        }

        if (rc < 0 && errno == EINTR) {
          continue;
        }
        if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          std::this_thread::sleep_for(1ms);
          continue;
        }
        return false;
      }
      return true;
    }
  }  // namespace

  serial_client_t::serial_client_t(std::string port, int baud, std::string mode, std::string delivery_policy):
      port_(std::move(port)),
      baud_(baud),
      mode_(std::move(mode)),
      delivery_policy_(std::move(delivery_policy)),
      running_(true),
      worker_(new std::thread([this]() {
        writer_loop();
      })) {
    send_mode_init();
  }

  serial_client_t::~serial_client_t() {
    {
      std::lock_guard lock(queue_mutex_);
      running_ = false;
    }
    queue_cv_.notify_all();

    if (worker_ && worker_->joinable()) {
      worker_->join();
    }

    delete worker_;
    worker_ = nullptr;
    close_port();
  }

  void serial_client_t::send_mode_init() {
    if (!mode_.empty()) {
      nlohmann::json payload {
        {"action", "set_mode"},
        {"mode", mode_},
      };
      enqueue_command(to_json_line(payload));
    }

    if (!delivery_policy_.empty()) {
      nlohmann::json payload {
        {"action", "set_delivery_policy"},
        {"delivery_policy", delivery_policy_},
      };
      enqueue_command(to_json_line(payload));
    }

    if (!delivery_policy_.empty()) {
      nlohmann::json payload {
        {"action", "set_input_policy"},
        {"input_policy", normalized_input_policy(delivery_policy_)},
      };
      enqueue_command(to_json_line(payload));
    }
  }

  void serial_client_t::send_button(std::string_view button, bool pressed) {
    nlohmann::json payload {
      {"action", pressed ? "button_press" : "button_release"},
      {"button", std::string(button)},
    };
    enqueue_command(to_json_line(payload));
  }

  void serial_client_t::send_stick(std::string_view stick_id, std::int16_t x, std::int16_t y) {
    enqueue_stick(stick_id, x, y);
  }

  void serial_client_t::send_hat(std::string_view direction) {
    enqueue_hat(direction);
  }

  void serial_client_t::enqueue_command(const std::string &line) {
    {
      std::lock_guard lock(queue_mutex_);
      if (!running_) {
        return;
      }

      // Cap queue growth during temporary disconnects.
      constexpr std::size_t max_queue = 4096;
      if (queue_.size() >= max_queue) {
        queue_.pop_front();
      }

      queue_.push_back(line);
    }

    queue_cv_.notify_one();
  }

  void serial_client_t::enqueue_hat(std::string_view direction) {
    {
      std::lock_guard lock(queue_mutex_);
      if (!running_) {
        return;
      }

      if (!pending_hat_dirty_ && pending_hat_ == direction) {
        return;
      }

      pending_hat_ = std::string(direction);
      pending_hat_dirty_ = true;
    }

    queue_cv_.notify_one();
  }

  void serial_client_t::enqueue_stick(std::string_view stick_id, std::int16_t x, std::int16_t y) {
    {
      std::lock_guard lock(queue_mutex_);
      if (!running_) {
        return;
      }

      auto &slot = (stick_id == "right"sv) ? pending_right_ : pending_left_;
      if (!slot.dirty && slot.x == x && slot.y == y) {
        return;
      }
      slot.x = x;
      slot.y = y;
      slot.dirty = true;
    }

    queue_cv_.notify_one();
  }

  void serial_client_t::writer_loop() {
    while (true) {
      std::string line;
      {
        std::unique_lock lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return !running_ || !queue_.empty() || pending_hat_dirty_ || pending_left_.dirty || pending_right_.dirty;
        });

        if (!running_ && queue_.empty() && !pending_hat_dirty_ && !pending_left_.dirty && !pending_right_.dirty) {
          break;
        }

        if (!queue_.empty()) {
          line = std::move(queue_.front());
          queue_.pop_front();
        } else if (pending_hat_dirty_) {
          line = make_hat_line(pending_hat_);
          pending_hat_dirty_ = false;
        } else if (pending_left_.dirty) {
          line = make_stick_line("left", pending_left_.x, pending_left_.y);
          pending_left_.dirty = false;
        } else if (pending_right_.dirty) {
          line = make_stick_line("right", pending_right_.x, pending_right_.y);
          pending_right_.dirty = false;
        } else {
          continue;
        }
      }

      if (!ensure_open()) {
        std::this_thread::sleep_for(500ms);
        continue;
      }

      if (!write_all(fd_, line.data(), line.size())) {
        BOOST_LOG(warning) << "ESP32 serial write failed on "sv << port_ << ": "sv << strerror(errno);
        close_port();
        std::this_thread::sleep_for(200ms);
      }
    }
  }

  bool serial_client_t::ensure_open() {
    if (fd_ >= 0) {
      return true;
    }

    auto fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      BOOST_LOG(debug) << "ESP32 serial port unavailable ["sv << port_ << "]: "sv << strerror(errno);
      return false;
    }

    termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
      BOOST_LOG(error) << "ESP32 serial tcgetattr failed for "sv << port_ << ": "sv << strerror(errno);
      ::close(fd);
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    const auto speed = baud_to_constant(baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      BOOST_LOG(error) << "ESP32 serial tcsetattr failed for "sv << port_ << ": "sv << strerror(errno);
      ::close(fd);
      return false;
    }

    // Switch to blocking writes.
    const auto flags = fcntl(fd, F_GETFL);
    if (flags >= 0) {
      fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    fd_ = fd;
    BOOST_LOG(info) << "ESP32 serial connected: "sv << port_ << " @ "sv << baud_;
    return true;
  }

  void serial_client_t::close_port() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string dpad_direction(std::uint32_t button_flags) {
    const bool up = button_flags & platf::DPAD_UP;
    const bool down = button_flags & platf::DPAD_DOWN;
    const bool left = button_flags & platf::DPAD_LEFT;
    const bool right = button_flags & platf::DPAD_RIGHT;

    // Firmware currently supports cardinal directions only.
    if (up && !down && !left && !right) {
      return "DUP"s;
    }
    if (down && !up && !left && !right) {
      return "DDOWN"s;
    }
    if (left && !right && !up && !down) {
      return "DLEFT"s;
    }
    if (right && !left && !up && !down) {
      return "DRIGHT"s;
    }

    return "center"s;
  }

  bool trigger_pressed(std::uint8_t value) {
    constexpr std::uint8_t trigger_threshold = 32;
    return value >= trigger_threshold;
  }
}  // namespace platf::esp32
