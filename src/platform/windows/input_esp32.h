/**
 * @file src/platform/windows/input_esp32.h
 * @brief Declarations for ESP32 serial controller transport on Windows.
 *
 * Uses the PABB (Pokémon Automation Bot-Base) binary protocol for controller
 * state updates (19-byte packets), with JSON used only for initial mode setup.
 */
#pragma once

// standard includes
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace platf::esp32 {
  class serial_client_t {
  public:
    serial_client_t(std::string port, int baud, std::string mode, std::string delivery_policy);
    ~serial_client_t();

    serial_client_t(const serial_client_t &) = delete;
    serial_client_t &operator=(const serial_client_t &) = delete;

    void send_mode_init();

    /**
     * @brief Send complete controller state as a PABB binary packet.
     * @param button_flags Moonlight button bitmask (platf:: constants).
     * @param lt Left trigger (0-255).
     * @param rt Right trigger (0-255).
     * @param lsX Left stick X (-32768 to 32767).
     * @param lsY Left stick Y (-32768 to 32767).
     * @param rsX Right stick X (-32768 to 32767).
     * @param rsY Right stick Y (-32768 to 32767).
     */
    void send_state(std::uint32_t button_flags, std::uint8_t lt, std::uint8_t rt,
                    std::int16_t lsX, std::int16_t lsY, std::int16_t rsX, std::int16_t rsY);

    void send_neutral_state();

  private:
    struct pabb_state_t {
      std::uint8_t buttons0 {0};
      std::uint8_t buttons1 {0};
      std::uint8_t dpad {8};  // centered
      std::uint8_t lx {0x80}, ly {0x80};
      std::uint8_t rx {0x80}, ry {0x80};
      bool dirty {false};
    };

    void writer_loop();
    bool ensure_open();
    void close_port();
    std::string build_pabb_packet(const pabb_state_t &state);

    std::string port_;
    int baud_;
    std::string mode_;
    std::string delivery_policy_;

    void *handle_ {nullptr};
    bool running_ {false};
    std::thread *worker_ {nullptr};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;  // JSON commands (mode init only)
    pabb_state_t state_;
    std::uint32_t seq_ {0};
  };
}  // namespace platf::esp32
