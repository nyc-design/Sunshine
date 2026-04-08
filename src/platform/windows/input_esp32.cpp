/**
 * @file src/platform/windows/input_esp32.cpp
 * @brief ESP32 serial controller transport on Windows using PABB binary protocol.
 *
 * Sends complete controller state as 19-byte PABB NS_WIRED_CONTROLLER_STATE
 * packets instead of individual JSON commands. JSON is only used for initial
 * mode/delivery policy setup.
 */

// standard includes
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

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
    // PABB protocol constants
    constexpr std::uint8_t PABB_MSG_CONTROLLER_STATE = 0x90;
    constexpr std::uint16_t PABB_HOLD_DURATION_MS = 100;  // safety timer: neutralize if no update in 100ms

    // CRC32C (Castagnoli) lookup table
    constexpr std::uint32_t crc32c_table[256] = {
      0x00000000, 0xF26B8303, 0xE13B70F7, 0x1350F3F4, 0xC79A971F, 0x3501141C, 0x2651B2E8, 0xD49A31EB,
      0x8AD958CF, 0x78B2DBCC, 0x6BE27D38, 0x9989FE3B, 0x4D439AD0, 0xBF2819D3, 0xAC78BF27, 0x5E133C24,
      0x105EC76F, 0xE235446C, 0xF165E298, 0x030E619B, 0xD7C40570, 0x25AF8673, 0x36FF2087, 0xC494A384,
      0x9AD7CAA0, 0x68BC49A3, 0x7BECEF57, 0x89876C54, 0x5D4D08BF, 0xAF268BBC, 0xBC762D48, 0x4E1DAE4B,
      0x20BD8EDE, 0xD2D60DDD, 0xC186AB29, 0x33ED282A, 0xE7274CC1, 0x154CCFC2, 0x061C6936, 0xF477EA35,
      0xAA34D311, 0x585F5012, 0x4B0FF6E6, 0xB96475E5, 0x6DAE110E, 0x9FC5920D, 0x8C9534F9, 0x7EFEB7FA,
      0x30B34CB1, 0xC2D8CFB2, 0xD1886946, 0x23E3EA45, 0xF72988AE, 0x05420BAD, 0x1612AD59, 0xE4792E5A,
      0xBA3A477E, 0x4851C47D, 0x5B016289, 0xA96AE18A, 0x7DA08561, 0x8FCB0662, 0x9C9BA096, 0x6EF02395,
      0x417B1DBC, 0xB3109EBF, 0xA040384B, 0x522BBB48, 0x86E1DFA3, 0x748A5CA0, 0x67DAFA54, 0x95B17957,
      0xCBF21073, 0x39999370, 0x2AC93584, 0xD8A2B687, 0x0C68D26C, 0xFE03516F, 0xED53F79B, 0x1F387498,
      0x51758FD3, 0xA31E0CD0, 0xB04EAA24, 0x42252927, 0x96EF4DCC, 0x6484CECF, 0x77D4683B, 0x85BFEB38,
      0xDBFC821C, 0x2997011F, 0x3AC7A7EB, 0xC8AC24E8, 0x1C664003, 0xEE0DC300, 0xFD5D65F4, 0x0F36E6F7,
      0x61C69362, 0x93AD1061, 0x80FDB695, 0x72963596, 0xA65C517D, 0x5437D27E, 0x4767748A, 0xB50CF789,
      0xEB4F9EAD, 0x19241DAE, 0x0A74BB5A, 0xF81F3859, 0x2CD55CB2, 0xDEBEDFB1, 0xCDEE7945, 0x3F85FA46,
      0x7148010D, 0x8323820E, 0x907324FA, 0x6218A7F9, 0xB6D2C312, 0x44B94011, 0x57E9E6E5, 0xA58265E6,
      0xFBC10CC2, 0x09AA8FC1, 0x1AFA2935, 0xE891AA36, 0x3C5BCEDD, 0xCE304DDE, 0xDD60EB2A, 0x2F0B6829,
      0x82F63B78, 0x709DB87B, 0x63CD1E8F, 0x91A69D8C, 0x456CF967, 0xB7077A64, 0xA457DC90, 0x56DC5F93,
      0x089F36B7, 0xFAF4B5B4, 0xE9A41340, 0x1BCF9043, 0xCF05F4A8, 0x3D6E77AB, 0x2E3ED15F, 0xDC55525C,
      0x92188917, 0x60730A14, 0x7323ACE0, 0x81482FE3, 0x55824B08, 0xA7E9C80B, 0xB4B96EFF, 0x46D2EDFC,
      0x189184D8, 0xEAFA07DB, 0xF9AAA12F, 0x0BC1222C, 0xDF0B46C7, 0x2D60C5C4, 0x3E306330, 0xCC5BE033,
      0xA2FB80A6, 0x509003A5, 0x43C0A551, 0xB1AB2652, 0x656142B9, 0x970AC1BA, 0x845A674E, 0x7631E44D,
      0x28728D69, 0xDA190E6A, 0xC949A89E, 0x3B222B9D, 0xEFE84F76, 0x1D83CC75, 0x0ED36A81, 0xFCB8E982,
      0xB2F512C9, 0x409E91CA, 0x53CE373E, 0xA1A5B43D, 0x756FD0D6, 0x870453D5, 0x9454F521, 0x663F7622,
      0x387C1F06, 0xCA179C05, 0xD9473AF1, 0x2B2CB9F2, 0xFF06DD19, 0x0D6D5E1A, 0x1E3DF8EE, 0xEC567BED,
      0xC2D7E58E, 0x30BC668D, 0x23ECC079, 0xD187437A, 0x054D2791, 0xF726A492, 0xE4760266, 0x161D8165,
      0x485EE841, 0xBA356B42, 0xA965CDB6, 0x5B0E4EB5, 0x8FC42A5E, 0x7DAFA95D, 0x6EFF0FA9, 0x9C948CAA,
      0xD2D977E1, 0x20B2F4E2, 0x33E25216, 0xC189D115, 0x1543B5FE, 0xE72836FD, 0xF4789009, 0x0613130A,
      0x58507A2E, 0xAA3BF92D, 0xB96B5FD9, 0x4B00DCDA, 0x9FCAB831, 0x6DA13B32, 0x7EF19DC6, 0x8C9A1EC5,
      0xE33A7E50, 0x1151FD53, 0x02015BA7, 0xF06AD8A4, 0x24A0BC4F, 0xD6CB3F4C, 0xC59B99B8, 0x37F01ABB,
      0x69B3739F, 0x9BD8F09C, 0x88885668, 0x7AE3D56B, 0xAE29B180, 0x5C423283, 0x4F129477, 0xBD791774,
      0xF334EC3F, 0x015F6F3C, 0x120FC9C8, 0xE0644ACB, 0x34AE2E20, 0xC6C5AD23, 0xD5950BD7, 0x279E88D4,
      0x79DDE1F0, 0x8BB662F3, 0x98E6C407, 0x6A8D4704, 0xBE4723EF, 0x4C2CA0EC, 0x5F7C0618, 0xAD17851B,
    };

    std::uint32_t crc32c(const std::uint8_t *data, std::size_t len) {
      std::uint32_t crc = 0xFFFFFFFF;
      for (std::size_t i = 0; i < len; ++i) {
        crc = crc32c_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
      }
      return crc ^ 0xFFFFFFFF;
    }

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

    // Convert Moonlight 16-bit axis to 8-bit (0x00-0xFF, 0x80=center)
    std::uint8_t axis_to_u8(std::int16_t value) {
      return static_cast<std::uint8_t>((static_cast<int>(value) + 32768) >> 8);
    }

    // Convert Moonlight dpad button flags to PABB hat value (0-7 directions, 8=centered)
    std::uint8_t dpad_to_hat(std::uint32_t flags) {
      const bool up = flags & platf::DPAD_UP;
      const bool down = flags & platf::DPAD_DOWN;
      const bool left = flags & platf::DPAD_LEFT;
      const bool right = flags & platf::DPAD_RIGHT;

      if (up && !down) {
        if (right && !left) return 1;      // up-right
        if (left && !right) return 7;      // up-left
        return 0;                          // up
      }
      if (down && !up) {
        if (right && !left) return 3;      // down-right
        if (left && !right) return 5;      // down-left
        return 4;                          // down
      }
      if (right && !left) return 2;        // right
      if (left && !right) return 6;        // left
      return 8;                            // centered
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
      std::lock_guard lock(mutex_);
      running_ = false;
    }
    cv_.notify_all();

    if (worker_ && worker_->joinable()) {
      worker_->join();
    }

    delete worker_;
    worker_ = nullptr;
    close_port();
  }

  void serial_client_t::send_mode_init() {
    std::lock_guard lock(mutex_);

    if (!mode_.empty()) {
      nlohmann::json payload {{"action", "set_mode"}, {"mode", mode_}};
      queue_.push_back(to_json_line(payload));
    }
    if (!delivery_policy_.empty()) {
      nlohmann::json payload {{"action", "set_delivery_policy"}, {"delivery_policy", delivery_policy_}};
      queue_.push_back(to_json_line(payload));
    }
    if (!delivery_policy_.empty()) {
      nlohmann::json payload {{"action", "set_input_policy"}, {"input_policy", normalized_input_policy(delivery_policy_)}};
      queue_.push_back(to_json_line(payload));
    }

    cv_.notify_one();
  }

  void serial_client_t::send_state(std::uint32_t button_flags, std::uint8_t lt, std::uint8_t rt,
                                   std::int16_t lsX, std::int16_t lsY, std::int16_t rsX, std::int16_t rsY) {
    // Map Moonlight buttons to PABB Switch Pro Controller byte layout.
    // Nintendo layout swap: Moonlight A (bottom) → Switch B, Moonlight B (right) → Switch A, etc.
    std::uint8_t b0 = 0;
    if (button_flags & platf::Y) b0 |= 0x01;     // Y → Y (bit 0)
    if (button_flags & platf::A) b0 |= 0x02;     // A (bottom, Xbox) → B (bottom, Switch) (bit 1)
    if (button_flags & platf::B) b0 |= 0x04;     // B (right, Xbox) → A (right, Switch) (bit 2)
    if (button_flags & platf::X) b0 |= 0x08;     // X (left, Xbox) → X (top, Switch) (bit 3)
    if (button_flags & platf::LEFT_BUTTON) b0 |= 0x10;   // LB → L (bit 4)
    if (button_flags & platf::RIGHT_BUTTON) b0 |= 0x20;  // RB → R (bit 5)
    if (lt >= 32) b0 |= 0x40;                             // LT → ZL (bit 6)
    if (rt >= 32) b0 |= 0x80;                             // RT → ZR (bit 7)

    std::uint8_t b1 = 0;
    if (button_flags & platf::BACK) b1 |= 0x01;          // Select → Minus (bit 0)
    if (button_flags & platf::START) b1 |= 0x02;         // Start → Plus (bit 1)
    if (button_flags & platf::LEFT_STICK) b1 |= 0x04;    // LStick → L3 (bit 2)
    if (button_flags & platf::RIGHT_STICK) b1 |= 0x08;   // RStick → R3 (bit 3)
    if (button_flags & platf::HOME) b1 |= 0x10;          // Home (bit 4)
    if (button_flags & platf::MISC_BUTTON) b1 |= 0x20;   // Capture (bit 5)

    {
      std::lock_guard lock(mutex_);
      if (!running_) return;

      state_.buttons0 = b0;
      state_.buttons1 = b1;
      state_.dpad = dpad_to_hat(button_flags);
      state_.lx = axis_to_u8(lsX);
      state_.ly = axis_to_u8(lsY);
      state_.rx = axis_to_u8(rsX);
      state_.ry = axis_to_u8(rsY);
      state_.dirty = true;
    }
    cv_.notify_one();
  }

  void serial_client_t::send_neutral_state() {
    send_state(0, 0, 0, 0, 0, 0, 0);
  }

  std::string serial_client_t::build_pabb_packet(const pabb_state_t &state) {
    // PABB NS_WIRED_CONTROLLER_STATE (0x90):
    //   [~len] [type] [seq:4LE] [duration:2LE] [b0] [b1] [dpad] [lx] [ly] [rx] [ry] [crc:4LE]
    //   Total: 19 bytes
    constexpr std::size_t packet_len = 19;
    std::array<std::uint8_t, packet_len> pkt {};

    pkt[0] = static_cast<std::uint8_t>(~packet_len);  // ~length
    pkt[1] = PABB_MSG_CONTROLLER_STATE;

    // Sequence number (4 bytes LE)
    const auto seq = seq_++;
    pkt[2] = static_cast<std::uint8_t>(seq);
    pkt[3] = static_cast<std::uint8_t>(seq >> 8);
    pkt[4] = static_cast<std::uint8_t>(seq >> 16);
    pkt[5] = static_cast<std::uint8_t>(seq >> 24);

    // Duration (2 bytes LE)
    pkt[6] = static_cast<std::uint8_t>(PABB_HOLD_DURATION_MS);
    pkt[7] = static_cast<std::uint8_t>(PABB_HOLD_DURATION_MS >> 8);

    // Controller state (7 bytes)
    pkt[8] = state.buttons0;
    pkt[9] = state.buttons1;
    pkt[10] = state.dpad;
    pkt[11] = state.lx;
    pkt[12] = state.ly;
    pkt[13] = state.rx;
    pkt[14] = state.ry;

    // CRC32C over bytes 0..14
    const auto crc = crc32c(pkt.data(), packet_len - 4);
    pkt[15] = static_cast<std::uint8_t>(crc);
    pkt[16] = static_cast<std::uint8_t>(crc >> 8);
    pkt[17] = static_cast<std::uint8_t>(crc >> 16);
    pkt[18] = static_cast<std::uint8_t>(crc >> 24);

    return std::string(reinterpret_cast<const char *>(pkt.data()), packet_len);
  }

  void serial_client_t::writer_loop() {
    while (true) {
      std::string text_batch;
      std::string binary_packet;

      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this]() {
          return !running_ || !queue_.empty() || state_.dirty;
        });

        if (!running_ && queue_.empty() && !state_.dirty) {
          break;
        }

        // Drain JSON queue (mode init commands)
        while (!queue_.empty()) {
          text_batch += queue_.front();
          queue_.pop_front();
        }

        // Build binary packet from latest state
        if (state_.dirty) {
          binary_packet = build_pabb_packet(state_);
          state_.dirty = false;
        }
      }

      if (text_batch.empty() && binary_packet.empty()) {
        continue;
      }

      if (!ensure_open()) {
        std::this_thread::sleep_for(500ms);
        continue;
      }

      auto serial = static_cast<HANDLE>(handle_);

      // Write JSON commands first (mode init), then binary state
      for (const auto &data : {text_batch, binary_packet}) {
        if (data.empty()) continue;

        std::size_t sent = 0;
        while (sent < data.size()) {
          DWORD written = 0;
          const BOOL ok = WriteFile(serial, data.data() + sent, static_cast<DWORD>(data.size() - sent), &written, nullptr);
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
  }

  bool serial_client_t::ensure_open() {
    if (handle_) {
      return true;
    }

    auto port_name = normalize_port_name(port_);
    auto serial = CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
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
      CloseHandle(static_cast<HANDLE>(handle_));
      handle_ = nullptr;
    }
  }
}  // namespace platf::esp32
