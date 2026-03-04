/**
 * @file src/platform/linux/uvcgrab.cpp
 * @brief Linux UVC capture backend implementation.
 */

// standard includes
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <poll.h>
#include <regex>
#include <set>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

// platform includes
#include <fcntl.h>
#include <linux/videodev2.h>

// local includes
#include "uvcgrab.h"
#include "src/logging.h"
#include "src/utility.h"

using namespace std::literals;
namespace fs = std::filesystem;

namespace platf::uvc {
  namespace {
    constexpr std::string_view V4L_BY_ID_DIR = "/dev/v4l/by-id"sv;
    constexpr std::string_view DEV_DIR = "/dev"sv;

    bool xioctl(int fd, unsigned long request, void *arg) {
      while (true) {
        if (ioctl(fd, request, arg) == 0) {
          return true;
        }
        if (errno != EINTR) {
          return false;
        }
      }
    }

    std::string make_display_name(const fs::path &stable_id) {
      auto name = stable_id.filename().string();
      if (name.empty()) {
        name = stable_id.string();
      }

      // Trim common udev suffix for readability.
      constexpr std::string_view suffix = "-video-index0"sv;
      if (name.size() > suffix.size() && name.ends_with(suffix)) {
        name.erase(name.size() - suffix.size());
      }

      std::replace(name.begin(), name.end(), '_', ' ');
      return name;
    }

    bool is_capture_device(const fs::path &device_path) {
      auto fd = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK);
      if (fd < 0) {
        return false;
      }

      auto close_guard = util::fail_guard([fd]() {
        ::close(fd);
      });

      v4l2_capability cap {};
      if (!xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        return false;
      }

      auto caps = cap.capabilities;
      if (caps & V4L2_CAP_DEVICE_CAPS) {
        caps = cap.device_caps;
      }

      const bool has_capture = (caps & V4L2_CAP_VIDEO_CAPTURE) || (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
      const bool has_streaming = caps & V4L2_CAP_STREAMING;
      return has_capture && has_streaming;
    }

    std::vector<fs::path> enumerate_candidate_paths() {
      std::vector<fs::path> candidates;
      std::set<std::string> seen;

      std::error_code ec;
      if (fs::exists(V4L_BY_ID_DIR, ec)) {
        for (const auto &entry : fs::directory_iterator(V4L_BY_ID_DIR, ec)) {
          if (ec || !entry.is_symlink(ec)) {
            continue;
          }

          auto path = entry.path();
          const auto inserted = seen.insert(path.string()).second;
          if (inserted) {
            candidates.emplace_back(std::move(path));
          }
        }
      }

      // Fallback or complement with /dev/video* nodes.
      static const std::regex video_regex {R"(^video[0-9]+$)"};
      for (const auto &entry : fs::directory_iterator(DEV_DIR, ec)) {
        if (ec || !entry.is_character_file(ec)) {
          continue;
        }

        auto filename = entry.path().filename().string();
        if (!std::regex_match(filename, video_regex)) {
          continue;
        }

        const auto key = entry.path().string();
        const auto inserted = seen.insert(key).second;
        if (inserted) {
          candidates.emplace_back(entry.path());
        }
      }

      std::sort(candidates.begin(), candidates.end());
      return candidates;
    }
  }  // namespace

  std::vector<device_info_t> enumerate_devices() {
    std::vector<device_info_t> devices;
    std::set<std::string> seen_device_paths;

    for (const auto &candidate : enumerate_candidate_paths()) {
      std::error_code ec;
      auto resolved = fs::weakly_canonical(candidate, ec);
      if (ec || resolved.empty()) {
        resolved = candidate;
      }

      if (!is_capture_device(resolved)) {
        continue;
      }

      const auto resolved_str = resolved.string();
      if (!seen_device_paths.insert(resolved_str).second) {
        continue;
      }

      device_info_t device;
      device.id = candidate.string();
      device.display_name = make_display_name(candidate);
      device.device_path = resolved_str;
      devices.emplace_back(std::move(device));
    }

    return devices;
  }
}  // namespace platf::uvc

namespace platf {
  namespace {
    struct uvc_img_t: public img_t {
      std::vector<std::uint8_t> backing;
    };

    struct mapped_buffer_t {
      void *data {};
      std::size_t size {};
    };

    bool xioctl(int fd, unsigned long request, void *arg) {
      while (true) {
        if (ioctl(fd, request, arg) == 0) {
          return true;
        }
        if (errno != EINTR) {
          return false;
        }
      }
    }

    std::uint8_t clamp_u8(int value) {
      return static_cast<std::uint8_t>(std::clamp(value, 0, 255));
    }

    void yuv_to_bgr0(int y, int u, int v, std::uint8_t *dst) {
      const int c = y - 16;
      const int d = u - 128;
      const int e = v - 128;

      const int r = (298 * c + 409 * e + 128) >> 8;
      const int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
      const int b = (298 * c + 516 * d + 128) >> 8;

      dst[0] = clamp_u8(b);
      dst[1] = clamp_u8(g);
      dst[2] = clamp_u8(r);
      dst[3] = 0;
    }

    class uvc_display_t: public display_t {
    public:
      ~uvc_display_t() override {
        stop();
      }

      int init(std::string source_name, std::string device_path, const video::config_t &config) {
        source_name_ = std::move(source_name);
        device_path_ = std::move(device_path);

        delay_ = std::chrono::nanoseconds {1s} / std::max(1, config.framerate);

        fd_ = ::open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
          BOOST_LOG(error) << "UVC: failed to open "sv << device_path_ << ": "sv << strerror(errno);
          return -1;
        }

        v4l2_capability cap {};
        if (!xioctl(fd_, VIDIOC_QUERYCAP, &cap)) {
          BOOST_LOG(error) << "UVC: VIDIOC_QUERYCAP failed for "sv << device_path_ << ": "sv << strerror(errno);
          return -1;
        }

        auto caps = cap.capabilities;
        if (caps & V4L2_CAP_DEVICE_CAPS) {
          caps = cap.device_caps;
        }

        if (!((caps & V4L2_CAP_VIDEO_CAPTURE) || (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
          BOOST_LOG(error) << "UVC: "sv << device_path_ << " is not a video capture device"sv;
          return -1;
        }

        if (!(caps & V4L2_CAP_STREAMING)) {
          BOOST_LOG(error) << "UVC: "sv << device_path_ << " does not support streaming io"sv;
          return -1;
        }

        if (configure_format(config)) {
          return -1;
        }

        if (configure_buffers()) {
          return -1;
        }

        BOOST_LOG(info) << "UVC: capturing from "sv << source_name_ << " ["sv << device_path_ << "] "sv << width << 'x' << height;
        return 0;
      }

      capture_e capture(const push_captured_image_cb_t &push_captured_image_cb, const pull_free_image_cb_t &pull_free_image_cb, bool * /*cursor*/) override {
        auto next_frame = std::chrono::steady_clock::now();
        sleep_overshoot_logger.reset();

        while (true) {
          auto now = std::chrono::steady_clock::now();
          if (next_frame > now) {
            std::this_thread::sleep_for(next_frame - now);
            sleep_overshoot_logger.first_point(next_frame);
            sleep_overshoot_logger.second_point_now_and_log();
          }

          next_frame += delay_;
          if (next_frame < now) {
            next_frame = now + delay_;
          }

          std::shared_ptr<img_t> img_out;
          if (!pull_free_image_cb(img_out)) {
            return capture_e::interrupted;
          }

          auto status = capture_frame(img_out.get(), 1000ms);
          switch (status) {
            case capture_e::ok:
              if (!push_captured_image_cb(std::move(img_out), true)) {
                return capture_e::ok;
              }
              break;
            case capture_e::timeout:
              if (!push_captured_image_cb(std::move(img_out), false)) {
                return capture_e::ok;
              }
              break;
            case capture_e::interrupted:
            case capture_e::reinit:
            case capture_e::error:
              return status;
          }
        }

        return capture_e::ok;
      }

      std::shared_ptr<img_t> alloc_img() override {
        auto img = std::make_shared<uvc_img_t>();
        img->width = width;
        img->height = height;
        img->pixel_pitch = 4;
        img->row_pitch = width * img->pixel_pitch;
        img->backing.resize(static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height));
        img->data = img->backing.data();
        return img;
      }

      int dummy_img(img_t *img) override {
        if (!img || !img->data) {
          return -1;
        }

        const auto bytes = static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height);
        std::fill_n(img->data, bytes, 0);
        img->frame_timestamp = std::chrono::steady_clock::now();
        return 0;
      }

      std::unique_ptr<avcodec_encode_device_t> make_avcodec_encode_device(pix_fmt_e /*pix_fmt*/) override {
        // Software conversion path in video.cpp can consume BGR0 frames directly.
        return std::make_unique<avcodec_encode_device_t>();
      }

    private:
      int configure_format(const video::config_t &config) {
        v4l2_format fmt {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (!xioctl(fd_, VIDIOC_G_FMT, &fmt)) {
          BOOST_LOG(error) << "UVC: VIDIOC_G_FMT failed: "sv << strerror(errno);
          return -1;
        }

        constexpr std::array preferred_formats {
#ifdef V4L2_PIX_FMT_BGR32
          V4L2_PIX_FMT_BGR32,
#endif
#ifdef V4L2_PIX_FMT_XBGR32
          V4L2_PIX_FMT_XBGR32,
#endif
          V4L2_PIX_FMT_BGR24,
          V4L2_PIX_FMT_RGB24,
          V4L2_PIX_FMT_YUYV,
          V4L2_PIX_FMT_UYVY,
        };

        bool format_selected = false;
        for (auto pix_fmt : preferred_formats) {
          v4l2_format trial = fmt;
          trial.fmt.pix.pixelformat = pix_fmt;
          if (config.width > 0) {
            trial.fmt.pix.width = config.width;
          }
          if (config.height > 0) {
            trial.fmt.pix.height = config.height;
          }

          if (!xioctl(fd_, VIDIOC_S_FMT, &trial)) {
            continue;
          }

          if (trial.fmt.pix.pixelformat == pix_fmt) {
            fmt = trial;
            format_selected = true;
            break;
          }
        }

        if (!format_selected) {
          if (!xioctl(fd_, VIDIOC_G_FMT, &fmt)) {
            BOOST_LOG(error) << "UVC: unable to query active pixel format"sv;
            return -1;
          }
        }

        pixel_format_ = fmt.fmt.pix.pixelformat;
        width = static_cast<int>(fmt.fmt.pix.width);
        height = static_cast<int>(fmt.fmt.pix.height);
        env_width = width;
        env_height = height;
        logical_width = width;
        logical_height = height;
        env_logical_width = width;
        env_logical_height = height;

        bytes_per_line_ = fmt.fmt.pix.bytesperline;
        if (bytes_per_line_ == 0) {
          switch (pixel_format_) {
#ifdef V4L2_PIX_FMT_BGR32
            case V4L2_PIX_FMT_BGR32:
#endif
#ifdef V4L2_PIX_FMT_XBGR32
            case V4L2_PIX_FMT_XBGR32:
#endif
              bytes_per_line_ = static_cast<std::uint32_t>(width * 4);
              break;
            case V4L2_PIX_FMT_BGR24:
            case V4L2_PIX_FMT_RGB24:
              bytes_per_line_ = static_cast<std::uint32_t>(width * 3);
              break;
            case V4L2_PIX_FMT_YUYV:
            case V4L2_PIX_FMT_UYVY:
              bytes_per_line_ = static_cast<std::uint32_t>(width * 2);
              break;
            default:
              break;
          }
        }

        if (!is_supported_pixel_format(pixel_format_)) {
          BOOST_LOG(error) << "UVC: unsupported pixel format 0x"sv << util::hex(pixel_format_).to_string_view();
          return -1;
        }

        return 0;
      }

      int configure_buffers() {
        v4l2_requestbuffers req {};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (!xioctl(fd_, VIDIOC_REQBUFS, &req)) {
          BOOST_LOG(error) << "UVC: VIDIOC_REQBUFS failed: "sv << strerror(errno);
          return -1;
        }

        if (req.count < 2) {
          BOOST_LOG(error) << "UVC: insufficient capture buffers"sv;
          return -1;
        }

        buffers_.resize(req.count);

        for (std::size_t i = 0; i < buffers_.size(); ++i) {
          v4l2_buffer buf {};
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
          buf.index = static_cast<std::uint32_t>(i);

          if (!xioctl(fd_, VIDIOC_QUERYBUF, &buf)) {
            BOOST_LOG(error) << "UVC: VIDIOC_QUERYBUF failed: "sv << strerror(errno);
            return -1;
          }

          buffers_[i].size = buf.length;
          buffers_[i].data = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, static_cast<off_t>(buf.m.offset));
          if (buffers_[i].data == MAP_FAILED) {
            BOOST_LOG(error) << "UVC: mmap failed: "sv << strerror(errno);
            buffers_[i].data = nullptr;
            return -1;
          }
        }

        for (std::size_t i = 0; i < buffers_.size(); ++i) {
          v4l2_buffer buf {};
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
          buf.index = static_cast<std::uint32_t>(i);

          if (!xioctl(fd_, VIDIOC_QBUF, &buf)) {
            BOOST_LOG(error) << "UVC: VIDIOC_QBUF failed: "sv << strerror(errno);
            return -1;
          }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (!xioctl(fd_, VIDIOC_STREAMON, &type)) {
          BOOST_LOG(error) << "UVC: VIDIOC_STREAMON failed: "sv << strerror(errno);
          return -1;
        }

        streaming_ = true;
        return 0;
      }

      capture_e capture_frame(img_t *img, std::chrono::milliseconds timeout) {
        if (!img || !img->data) {
          return capture_e::error;
        }

        pollfd pfd {};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        int poll_status;
        do {
          poll_status = poll(&pfd, 1, static_cast<int>(timeout.count()));
        } while (poll_status < 0 && errno == EINTR);

        if (poll_status == 0) {
          return capture_e::timeout;
        }

        if (poll_status < 0) {
          BOOST_LOG(error) << "UVC: poll failed: "sv << strerror(errno);
          return capture_e::error;
        }

        if (!(pfd.revents & POLLIN)) {
          return capture_e::timeout;
        }

        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (!xioctl(fd_, VIDIOC_DQBUF, &buf)) {
          if (errno == EAGAIN) {
            return capture_e::timeout;
          }

          BOOST_LOG(error) << "UVC: VIDIOC_DQBUF failed: "sv << strerror(errno);
          return capture_e::reinit;
        }

        const auto requeue = util::fail_guard([&]() {
          if (!xioctl(fd_, VIDIOC_QBUF, &buf)) {
            BOOST_LOG(error) << "UVC: VIDIOC_QBUF failed: "sv << strerror(errno);
          }
        });

        if (buf.index >= buffers_.size()) {
          BOOST_LOG(error) << "UVC: buffer index out of range"sv;
          return capture_e::error;
        }

        auto *src = static_cast<const std::uint8_t *>(buffers_[buf.index].data);
        if (!src) {
          return capture_e::error;
        }

        if (convert_to_bgr0(src, buf.bytesused, img)) {
          return capture_e::error;
        }

        img->frame_timestamp = std::chrono::steady_clock::now();
        return capture_e::ok;
      }

      int convert_to_bgr0(const std::uint8_t *src, std::size_t bytes_used, img_t *img) {
        if (!src || !img || !img->data) {
          return -1;
        }

        const auto dst_stride = static_cast<std::size_t>(img->row_pitch);
        const auto src_stride = static_cast<std::size_t>(bytes_per_line_);
        auto *dst_base = img->data;

        switch (pixel_format_) {
#ifdef V4L2_PIX_FMT_BGR32
          case V4L2_PIX_FMT_BGR32:
#endif
#ifdef V4L2_PIX_FMT_XBGR32
          case V4L2_PIX_FMT_XBGR32:
#endif
            {
              const auto line_bytes = static_cast<std::size_t>(width) * 4;
              for (int y = 0; y < height; ++y) {
                const auto src_off = static_cast<std::size_t>(y) * src_stride;
                const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
                if (src_off + line_bytes > bytes_used) {
                  break;
                }
                std::memcpy(dst_base + dst_off, src + src_off, line_bytes);
              }
              return 0;
            }

          case V4L2_PIX_FMT_BGR24:
            {
              for (int y = 0; y < height; ++y) {
                const auto src_off = static_cast<std::size_t>(y) * src_stride;
                const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
                const auto *line = src + src_off;
                auto *out = dst_base + dst_off;

                for (int x = 0; x < width; ++x) {
                  const auto px_off = static_cast<std::size_t>(x) * 3;
                  out[x * 4 + 0] = line[px_off + 0];
                  out[x * 4 + 1] = line[px_off + 1];
                  out[x * 4 + 2] = line[px_off + 2];
                  out[x * 4 + 3] = 0;
                }
              }
              return 0;
            }

          case V4L2_PIX_FMT_RGB24:
            {
              for (int y = 0; y < height; ++y) {
                const auto src_off = static_cast<std::size_t>(y) * src_stride;
                const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
                const auto *line = src + src_off;
                auto *out = dst_base + dst_off;

                for (int x = 0; x < width; ++x) {
                  const auto px_off = static_cast<std::size_t>(x) * 3;
                  out[x * 4 + 0] = line[px_off + 2];
                  out[x * 4 + 1] = line[px_off + 1];
                  out[x * 4 + 2] = line[px_off + 0];
                  out[x * 4 + 3] = 0;
                }
              }
              return 0;
            }

          case V4L2_PIX_FMT_YUYV:
            {
              for (int y = 0; y < height; ++y) {
                const auto src_off = static_cast<std::size_t>(y) * src_stride;
                const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
                const auto *line = src + src_off;
                auto *out = dst_base + dst_off;

                for (int x = 0; x < width; x += 2) {
                  const auto px_off = static_cast<std::size_t>(x) * 2;
                  const int y0 = line[px_off + 0];
                  const int u = line[px_off + 1];
                  const int y1 = line[px_off + 2];
                  const int v = line[px_off + 3];

                  yuv_to_bgr0(y0, u, v, out + static_cast<std::size_t>(x) * 4);
                  if (x + 1 < width) {
                    yuv_to_bgr0(y1, u, v, out + static_cast<std::size_t>(x + 1) * 4);
                  }
                }
              }
              return 0;
            }

          case V4L2_PIX_FMT_UYVY:
            {
              for (int y = 0; y < height; ++y) {
                const auto src_off = static_cast<std::size_t>(y) * src_stride;
                const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
                const auto *line = src + src_off;
                auto *out = dst_base + dst_off;

                for (int x = 0; x < width; x += 2) {
                  const auto px_off = static_cast<std::size_t>(x) * 2;
                  const int u = line[px_off + 0];
                  const int y0 = line[px_off + 1];
                  const int v = line[px_off + 2];
                  const int y1 = line[px_off + 3];

                  yuv_to_bgr0(y0, u, v, out + static_cast<std::size_t>(x) * 4);
                  if (x + 1 < width) {
                    yuv_to_bgr0(y1, u, v, out + static_cast<std::size_t>(x + 1) * 4);
                  }
                }
              }
              return 0;
            }

          default:
            BOOST_LOG(error) << "UVC: unsupported pixel format conversion for 0x"sv << util::hex(pixel_format_).to_string_view();
            return -1;
        }
      }

      bool is_supported_pixel_format(std::uint32_t pix_fmt) const {
        switch (pix_fmt) {
#ifdef V4L2_PIX_FMT_BGR32
          case V4L2_PIX_FMT_BGR32:
#endif
#ifdef V4L2_PIX_FMT_XBGR32
          case V4L2_PIX_FMT_XBGR32:
#endif
          case V4L2_PIX_FMT_BGR24:
          case V4L2_PIX_FMT_RGB24:
          case V4L2_PIX_FMT_YUYV:
          case V4L2_PIX_FMT_UYVY:
            return true;
          default:
            return false;
        }
      }

      void stop() {
        if (fd_ >= 0 && streaming_) {
          v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          if (!xioctl(fd_, VIDIOC_STREAMOFF, &type)) {
            BOOST_LOG(debug) << "UVC: VIDIOC_STREAMOFF failed: "sv << strerror(errno);
          }
          streaming_ = false;
        }

        for (auto &buffer : buffers_) {
          if (buffer.data) {
            munmap(buffer.data, buffer.size);
            buffer.data = nullptr;
            buffer.size = 0;
          }
        }
        buffers_.clear();

        if (fd_ >= 0) {
          ::close(fd_);
          fd_ = -1;
        }
      }

      std::string source_name_;
      std::string device_path_;
      int fd_ {-1};
      bool streaming_ {false};
      std::vector<mapped_buffer_t> buffers_;

      std::chrono::nanoseconds delay_ {16666666ns};
      std::uint32_t pixel_format_ {};
      std::uint32_t bytes_per_line_ {};
    };

    const uvc::device_info_t *find_device(std::string_view display_name, const std::vector<uvc::device_info_t> &devices) {
      for (const auto &device : devices) {
        if (display_name.empty()) {
          return &device;
        }

        auto id_filename = fs::path(device.id).filename().string();
        if (display_name == device.id || display_name == device.device_path || display_name == device.display_name || display_name == id_filename) {
          return &device;
        }
      }

      return nullptr;
    }
  }  // namespace

  std::vector<std::string> uvc_display_names() {
    std::vector<std::string> result;
    for (const auto &device : uvc::enumerate_devices()) {
      result.emplace_back(device.id);
    }
    return result;
  }

  std::shared_ptr<display_t> uvc_display(mem_type_e hwdevice_type, const std::string &display_name, const video::config_t &config) {
    if (hwdevice_type != mem_type_e::system && hwdevice_type != mem_type_e::vaapi && hwdevice_type != mem_type_e::cuda) {
      BOOST_LOG(error) << "UVC: unsupported hw device type requested"sv;
      return nullptr;
    }

    auto devices = uvc::enumerate_devices();
    if (devices.empty()) {
      BOOST_LOG(error) << "UVC: no capture card devices found"sv;
      return nullptr;
    }

    const auto *device = find_device(display_name, devices);
    if (!device) {
      BOOST_LOG(error) << "UVC: requested source not found: "sv << display_name;
      return nullptr;
    }

    auto display = std::make_shared<uvc_display_t>();
    if (display->init(device->id, device->device_path, config)) {
      return nullptr;
    }

    return display;
  }
}  // namespace platf
