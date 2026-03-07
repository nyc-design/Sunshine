/**
 * @file src/platform/windows/input_esp32.cpp
 * @brief Definitions for ESP32 serial controller transport on Windows.
 */

// standard includes
#include <algorithm>
#include <cctype>
#include <chrono>

// platform includes
#ifndef NOMINMAX
  #define NOMINMAX
#endif
#include <Windows.h>

// lib includes
#include <boost/algorithm/string/predicate.hpp>
#include <nlohmann/json.hpp>

// local includes
#include "input_esp32.h"
#include "src/logging.h"
#include "src/platform/common.h"

using namespace std::literals;

namespace platf::esp32 {
  namespace {
    std::string normalize_port_name(const std::string &port) {
      if (port.rfind("\\\\.\\"s, 0) == 0) {
        return port;
      }

      if (boost::algorithm::istarts_with(port, "COM"s)) {
        return "\\\\.\\"s + port;
      }

      return port;
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

      // Backward compatibility with old config wording.
      if (policy == "bluetooth"sv || policy == "bt"sv) {
        return "websocket"s;
      }
      if (policy == "wifi"sv) {
        return "http"s;
      }

      return policy;
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
      std::string batch;
      {
        std::unique_lock lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return !running_ || !queue_.empty() || pending_hat_dirty_ || pending_left_.dirty || pending_right_.dirty;
        });

        if (!running_ && queue_.empty() && !pending_hat_dirty_ && !pending_left_.dirty && !pending_right_.dirty) {
          break;
        }

        // Drain all pending items into a single batch for one write syscall.
        while (!queue_.empty()) {
          batch += queue_.front();
          queue_.pop_front();
        }
        if (pending_hat_dirty_) {
          batch += make_hat_line(pending_hat_);
          pending_hat_dirty_ = false;
        }
        if (pending_left_.dirty) {
          batch += make_stick_line("left", pending_left_.x, pending_left_.y);
          pending_left_.dirty = false;
        }
        if (pending_right_.dirty) {
          batch += make_stick_line("right", pending_right_.x, pending_right_.y);
          pending_right_.dirty = false;
        }
      }

      if (batch.empty()) {
        continue;
      }

      if (!ensure_open()) {
        std::this_thread::sleep_for(500ms);
        continue;
      }

      auto serial = static_cast<HANDLE>(handle_);
      std::size_t sent = 0;
      while (sent < batch.size()) {
        DWORD written = 0;
        const BOOL ok = WriteFile(serial, batch.data() + sent, static_cast<DWORD>(batch.size() - sent), &written, nullptr);
        if (!ok) {
          BOOST_LOG(warning) << "ESP32 serial write failed on "sv << port_ << " [error="sv << GetLastError() << ']';
          close_port();
          std::this_thread::sleep_for(200ms);
          break;
        }

        if (written == 0) {
          std::this_thread::sleep_for(1ms);
          continue;
        }

        sent += written;
      }
    }
  }

  bool serial_client_t::ensure_open() {
    if (handle_) {
      return true;
    }

    auto port_name = normalize_port_name(port_);
    auto serial = CreateFileA(
      port_name.c_str(),
      GENERIC_READ | GENERIC_WRITE,
      0,
      nullptr,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL,
      nullptr
    );

    if (serial == INVALID_HANDLE_VALUE) {
      BOOST_LOG(debug) << "ESP32 serial port unavailable ["sv << port_ << "] [error="sv << GetLastError() << ']';
      return false;
    }

    DCB dcb {};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(serial, &dcb)) {
      BOOST_LOG(error) << "ESP32 GetCommState failed for "sv << port_ << " [error="sv << GetLastError() << ']';
      CloseHandle(serial);
      return false;
    }

    dcb.BaudRate = static_cast<DWORD>(std::clamp(baud_, 1200, 4000000));
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(serial, &dcb)) {
      BOOST_LOG(error) << "ESP32 SetCommState failed for "sv << port_ << " [error="sv << GetLastError() << ']';
      CloseHandle(serial);
      return false;
    }

    COMMTIMEOUTS timeouts {};
    timeouts.WriteTotalTimeoutConstant = 2000;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(serial, &timeouts);

    PurgeComm(serial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    handle_ = serial;
    BOOST_LOG(info) << "ESP32 serial connected: "sv << port_ << " @ "sv << dcb.BaudRate;
    return true;
  }

  void serial_client_t::close_port() {
    if (handle_) {
      auto serial = static_cast<HANDLE>(handle_);
      CloseHandle(serial);
      handle_ = nullptr;
    }
  }

  std::string dpad_direction(std::uint32_t button_flags) {
    const bool up = button_flags & platf::DPAD_UP;
    const bool down = button_flags & platf::DPAD_DOWN;
    const bool left = button_flags & platf::DPAD_LEFT;
    const bool right = button_flags & platf::DPAD_RIGHT;

    // Cardinal directions
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

    // Diagonal directions
    if (up && right && !down && !left) {
      return "DUPRIGHT"s;
    }
    if (up && left && !down && !right) {
      return "DUPLEFT"s;
    }
    if (down && right && !up && !left) {
      return "DDOWNRIGHT"s;
    }
    if (down && left && !up && !right) {
      return "DDOWNLEFT"s;
    }

    return "center"s;
  }

  bool trigger_pressed(std::uint8_t value) {
    constexpr std::uint8_t trigger_threshold = 32;
    return value >= trigger_threshold;
  }
}  // namespace platf::esp32
