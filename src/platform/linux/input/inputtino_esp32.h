/**
 * @file src/platform/linux/input/inputtino_esp32.h
 * @brief Declarations for ESP32 serial controller transport.
 */
#pragma once

// standard includes
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace platf::esp32 {
  class serial_client_t {
  public:
    serial_client_t(std::string port, int baud, std::string mode, std::string delivery_policy);
    ~serial_client_t();

    serial_client_t(const serial_client_t &) = delete;
    serial_client_t &operator=(const serial_client_t &) = delete;

    void send_mode_init();
    void send_button(std::string_view button, bool pressed);
    void send_stick(std::string_view stick_id, std::int16_t x, std::int16_t y);
    void send_hat(std::string_view direction);

  private:
    void enqueue_command(const std::string &line);
    void writer_loop();
    bool ensure_open();
    void close_port();

    std::string port_;
    int baud_;
    std::string mode_;
    std::string delivery_policy_;

    int fd_ {-1};
    bool running_ {false};
    std::thread *worker_ {nullptr};
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::string> queue_;
  };

  std::string dpad_direction(std::uint32_t button_flags);
  bool trigger_pressed(std::uint8_t value);
}  // namespace platf::esp32
