/**
 * @file src/platform/linux/input/inputtino_esp32.cpp
 * @brief ESP32 serial controller transport using PABB binary protocol.
 */

// standard includes
#include <algorithm>
#include <array>
#include <cerrno>
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
    constexpr std::uint8_t PABB_MSG_CONTROLLER_STATE = 0x90;
    constexpr std::uint16_t PABB_HOLD_DURATION_MS = 100;

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

    speed_t baud_to_constant(int baud) {
      switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B576000
        case 576000: return B576000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1152000
        case 1152000: return B1152000;
#endif
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
#ifdef B2500000
        case 2500000: return B2500000;
#endif
#ifdef B3000000
        case 3000000: return B3000000;
#endif
#ifdef B3500000
        case 3500000: return B3500000;
#endif
#ifdef B4000000
        case 4000000: return B4000000;
#endif
        default: return B115200;
      }
    }

    std::string to_json_line(const nlohmann::json &payload) {
      return payload.dump() + "\n";
    }

    std::string normalized_input_policy(std::string policy) {
      std::transform(policy.begin(), policy.end(), policy.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      if (policy == "bluetooth"sv || policy == "bt"sv) return "websocket"s;
      if (policy == "wifi"sv) return "http"s;
      return policy;
    }

    bool write_all(int fd, const char *data, std::size_t size) {
      std::size_t sent = 0;
      while (sent < size) {
        const auto rc = ::write(fd, data + sent, size - sent);
        if (rc > 0) { sent += static_cast<std::size_t>(rc); continue; }
        if (rc < 0 && errno == EINTR) continue;
        if (rc < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { std::this_thread::sleep_for(1ms); continue; }
        return false;
      }
      return true;
    }

    std::uint8_t axis_to_u8(std::int16_t value) {
      return static_cast<std::uint8_t>((static_cast<int>(value) + 32768) >> 8);
    }

    std::uint8_t dpad_to_hat(std::uint32_t flags) {
      const bool up = flags & platf::DPAD_UP;
      const bool down = flags & platf::DPAD_DOWN;
      const bool left = flags & platf::DPAD_LEFT;
      const bool right = flags & platf::DPAD_RIGHT;
      if (up && !down) { if (right && !left) return 1; if (left && !right) return 7; return 0; }
      if (down && !up) { if (right && !left) return 3; if (left && !right) return 5; return 4; }
      if (right && !left) return 2;
      if (left && !right) return 6;
      return 8;
    }
  }  // namespace

  serial_client_t::serial_client_t(std::string port, int baud, std::string mode, std::string delivery_policy):
      port_(std::move(port)), baud_(baud), mode_(std::move(mode)), delivery_policy_(std::move(delivery_policy)),
      running_(true),
      worker_(new std::thread([this]() { writer_loop(); })) {
    send_mode_init();
  }

  serial_client_t::~serial_client_t() {
    { std::lock_guard lock(mutex_); running_ = false; }
    cv_.notify_all();
    if (worker_ && worker_->joinable()) worker_->join();
    delete worker_;
    worker_ = nullptr;
    close_port();
  }

  void serial_client_t::send_mode_init() {
    std::lock_guard lock(mutex_);
    if (!mode_.empty()) {
      queue_.push_back(to_json_line(nlohmann::json {{"action", "set_mode"}, {"mode", mode_}}));
    }
    if (!delivery_policy_.empty()) {
      queue_.push_back(to_json_line(nlohmann::json {{"action", "set_delivery_policy"}, {"delivery_policy", delivery_policy_}}));
      queue_.push_back(to_json_line(nlohmann::json {{"action", "set_input_policy"}, {"input_policy", normalized_input_policy(delivery_policy_)}}));
    }
    cv_.notify_one();
  }

  void serial_client_t::send_state(std::uint32_t button_flags, std::uint8_t lt, std::uint8_t rt,
                                   std::int16_t lsX, std::int16_t lsY, std::int16_t rsX, std::int16_t rsY) {
    std::uint8_t b0 = 0;
    if (button_flags & platf::Y) b0 |= 0x01;
    if (button_flags & platf::A) b0 |= 0x02;  // Xbox A (bottom) → Switch B (bottom)
    if (button_flags & platf::B) b0 |= 0x04;  // Xbox B (right) → Switch A (right)
    if (button_flags & platf::X) b0 |= 0x08;  // Xbox X (left) → Switch X (top)
    if (button_flags & platf::LEFT_BUTTON) b0 |= 0x10;
    if (button_flags & platf::RIGHT_BUTTON) b0 |= 0x20;
    if (lt >= 32) b0 |= 0x40;
    if (rt >= 32) b0 |= 0x80;

    std::uint8_t b1 = 0;
    if (button_flags & platf::BACK) b1 |= 0x01;
    if (button_flags & platf::START) b1 |= 0x02;
    if (button_flags & platf::LEFT_STICK) b1 |= 0x04;
    if (button_flags & platf::RIGHT_STICK) b1 |= 0x08;
    if (button_flags & platf::HOME) b1 |= 0x10;
    if (button_flags & platf::MISC_BUTTON) b1 |= 0x20;

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
    constexpr std::size_t packet_len = 19;
    std::array<std::uint8_t, packet_len> pkt {};

    pkt[0] = static_cast<std::uint8_t>(~packet_len);
    pkt[1] = PABB_MSG_CONTROLLER_STATE;

    const auto seq = seq_++;
    pkt[2] = static_cast<std::uint8_t>(seq);
    pkt[3] = static_cast<std::uint8_t>(seq >> 8);
    pkt[4] = static_cast<std::uint8_t>(seq >> 16);
    pkt[5] = static_cast<std::uint8_t>(seq >> 24);

    pkt[6] = static_cast<std::uint8_t>(PABB_HOLD_DURATION_MS);
    pkt[7] = static_cast<std::uint8_t>(PABB_HOLD_DURATION_MS >> 8);

    pkt[8] = state.buttons0;
    pkt[9] = state.buttons1;
    pkt[10] = state.dpad;
    pkt[11] = state.lx;
    pkt[12] = state.ly;
    pkt[13] = state.rx;
    pkt[14] = state.ry;

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
        cv_.wait(lock, [this]() { return !running_ || !queue_.empty() || state_.dirty; });
        if (!running_ && queue_.empty() && !state_.dirty) break;

        while (!queue_.empty()) { text_batch += queue_.front(); queue_.pop_front(); }
        if (state_.dirty) { binary_packet = build_pabb_packet(state_); state_.dirty = false; }
      }

      if (text_batch.empty() && binary_packet.empty()) continue;

      if (!ensure_open()) { std::this_thread::sleep_for(500ms); continue; }

      for (const auto &data : {text_batch, binary_packet}) {
        if (data.empty()) continue;
        if (!write_all(fd_, data.data(), data.size())) {
          BOOST_LOG(warning) << "ESP32 serial write failed on "sv << port_ << ": "sv << strerror(errno);
          close_port();
          std::this_thread::sleep_for(200ms);
          break;
        }
      }
    }
  }

  bool serial_client_t::ensure_open() {
    if (fd_ >= 0) return true;

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

    const auto flags = fcntl(fd, F_GETFL);
    if (flags >= 0) fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    fd_ = fd;
    BOOST_LOG(info) << "ESP32 serial connected: "sv << port_ << " @ "sv << baud_;
    return true;
  }

  void serial_client_t::close_port() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }
}  // namespace platf::esp32
