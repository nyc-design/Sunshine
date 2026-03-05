/**
 * @file src/platform/windows/input_esp32.cpp
 * @brief Definitions for ESP32 serial controller transport on Windows.
 */

// standard includes
#include <algorithm>
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
  }

  void serial_client_t::send_button(std::string_view button, bool pressed) {
    nlohmann::json payload {
      {"action", pressed ? "button_press" : "button_release"},
      {"button", std::string(button)},
    };
    enqueue_command(to_json_line(payload));
  }

  void serial_client_t::send_stick(std::string_view stick_id, std::int16_t x, std::int16_t y) {
    nlohmann::json payload {
      {"action", "stick"},
      {"stick_id", std::string(stick_id)},
      {"x", x},
      {"y", y},
    };
    enqueue_command(to_json_line(payload));
  }

  void serial_client_t::send_hat(std::string_view direction) {
    nlohmann::json payload {
      {"action", "hat"},
      {"direction", std::string(direction)},
    };
    enqueue_command(to_json_line(payload));
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

  void serial_client_t::writer_loop() {
    while (true) {
      std::string line;
      {
        std::unique_lock lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return !running_ || !queue_.empty();
        });

        if (!running_ && queue_.empty()) {
          break;
        }

        line = std::move(queue_.front());
        queue_.pop_front();
      }

      if (!ensure_open()) {
        std::this_thread::sleep_for(500ms);
        continue;
      }

      auto serial = static_cast<HANDLE>(handle_);
      std::size_t sent = 0;
      while (sent < line.size()) {
        DWORD written = 0;
        const BOOL ok = WriteFile(serial, line.data() + sent, static_cast<DWORD>(line.size() - sent), &written, nullptr);
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

    if (up && !down) {
      return "up"s;
    }
    if (down && !up) {
      return "down"s;
    }
    if (left && !right) {
      return "left"s;
    }
    if (right && !left) {
      return "right"s;
    }

    return "center"s;
  }

  bool trigger_pressed(std::uint8_t value) {
    constexpr std::uint8_t trigger_threshold = 32;
    return value >= trigger_threshold;
  }
}  // namespace platf::esp32
