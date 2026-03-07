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

// lib includes
extern "C" {
#include <libswscale/swscale.h>
}

// local includes
#include "uvcgrab.h"
#include "src/logging.h"
#include "src/utility.h"
#include "src/video.h"

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

    class uvc_display_t: public display_t {
    public:
      ~uvc_display_t() override {
        stop();
      }

      int init(std::string source_name, std::string device_path, const video::config_t &config) {
        source_name_ = std::move(source_name);
        device_path_ = std::move(device_path);

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

        BOOST_LOG(info) << "UVC: capturing from "sv << source_name_ << " ["sv << device_path_ << "] "sv
                        << width << 'x' << height << " (passthrough=" << (nv12_passthrough_ ? "yes" : "no") << ')';
        return 0;
      }

      // Event-driven capture: poll waits for the device to deliver a frame.
      // No software rate limiting — the capture card IS the timing source.
      capture_e capture(const push_captured_image_cb_t &push_captured_image_cb, const pull_free_image_cb_t &pull_free_image_cb, bool * /*cursor*/) override {
        while (true) {
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

        if (nv12_passthrough_) {
          img->pixel_pitch = 1;
          img->row_pitch = width;
          img->source_format = img_t::nv12;
          const auto y_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
          const auto uv_size = static_cast<std::size_t>(width) * static_cast<std::size_t>((height + 1) / 2);
          img->backing.resize(y_size + uv_size);
          img->uv_offset = static_cast<std::int32_t>(y_size);
          img->uv_pitch = width;
        } else {
          img->pixel_pitch = 4;
          img->row_pitch = width * 4;
          img->source_format = img_t::bgr0;
          img->backing.resize(static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(height));
        }

        img->data = img->backing.data();
        return img;
      }

      int dummy_img(img_t *img) override {
        if (!img || !img->data) {
          return -1;
        }

        if (img->source_format == img_t::nv12) {
          const auto y_size = static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height);
          const auto uv_size = static_cast<std::size_t>(img->uv_pitch) * static_cast<std::size_t>((img->height + 1) / 2);
          std::fill_n(img->data, y_size, static_cast<std::uint8_t>(16));
          std::fill_n(img->data + img->uv_offset, uv_size, static_cast<std::uint8_t>(128));
        } else {
          const auto bytes = static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height);
          std::fill_n(img->data, bytes, 0);
        }

        img->frame_timestamp = std::chrono::steady_clock::now();
        return 0;
      }

      std::unique_ptr<avcodec_encode_device_t> make_avcodec_encode_device(pix_fmt_e /*pix_fmt*/) override {
        return std::make_unique<avcodec_encode_device_t>();
      }

      bool is_codec_supported(std::string_view /*name*/, const ::video::config_t & /*config*/) override {
        return true;
      }

    private:
      int configure_format(const video::config_t &config) {
        v4l2_format fmt {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (!xioctl(fd_, VIDIOC_G_FMT, &fmt)) {
          BOOST_LOG(error) << "UVC: VIDIOC_G_FMT failed: "sv << strerror(errno);
          return -1;
        }

        // NV12 first: encoder's native format, enables zero-conversion passthrough.
        constexpr std::array preferred_formats {
          V4L2_PIX_FMT_NV12,
#ifdef V4L2_PIX_FMT_BGR32
          V4L2_PIX_FMT_BGR32,
#endif
#ifdef V4L2_PIX_FMT_XBGR32
          V4L2_PIX_FMT_XBGR32,
#endif
          V4L2_PIX_FMT_YUYV,
          V4L2_PIX_FMT_UYVY,
          V4L2_PIX_FMT_BGR24,
          V4L2_PIX_FMT_RGB24,
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
            case V4L2_PIX_FMT_NV12:
              bytes_per_line_ = static_cast<std::uint32_t>(width);
              break;
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

        // NV12 passthrough: both native NV12 and YUV packed formats (converted to NV12) use this path.
        nv12_passthrough_ = (pixel_format_ == V4L2_PIX_FMT_NV12 || pixel_format_ == V4L2_PIX_FMT_YUYV || pixel_format_ == V4L2_PIX_FMT_UYVY);

        // Convert YUV packed formats directly to NV12 (single conversion) instead of
        // YUV→BGR0→NV12 (double conversion that causes stuttering).
        if (pixel_format_ == V4L2_PIX_FMT_YUYV) {
          sws_.reset(sws_getContext(width, height, AV_PIX_FMT_YUYV422, width, height, AV_PIX_FMT_NV12, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr));
          if (!sws_) {
            BOOST_LOG(error) << "UVC: failed to initialize YUYV→NV12 conversion context"sv;
            return -1;
          }
        } else if (pixel_format_ == V4L2_PIX_FMT_UYVY) {
          sws_.reset(sws_getContext(width, height, AV_PIX_FMT_UYVY422, width, height, AV_PIX_FMT_NV12, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr));
          if (!sws_) {
            BOOST_LOG(error) << "UVC: failed to initialize UYVY→NV12 conversion context"sv;
            return -1;
          }
        } else {
          sws_.reset();
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

        if (convert_frame(src, buf.bytesused, img)) {
          return capture_e::error;
        }

        img->frame_timestamp = std::chrono::steady_clock::now();
        return capture_e::ok;
      }

      int convert_frame(const std::uint8_t *src, std::size_t bytes_used, img_t *img) {
        if (!src || !img || !img->data) {
          return -1;
        }

        const auto src_stride = static_cast<std::size_t>(bytes_per_line_);

        // NV12 native: copy planes directly, no color conversion.
        if (pixel_format_ == V4L2_PIX_FMT_NV12) {
          const auto y_plane_size = src_stride * static_cast<std::size_t>(height);
          const auto uv_height = static_cast<std::size_t>((height + 1) / 2);
          const auto uv_plane_size = src_stride * uv_height;
          if (bytes_used < y_plane_size + uv_plane_size) {
            return -1;
          }

          const auto dst_y_stride = static_cast<std::size_t>(img->row_pitch);
          const auto line_bytes = static_cast<std::size_t>(width);

          // Y plane
          if (src_stride == dst_y_stride) {
            std::memcpy(img->data, src, y_plane_size);
          } else {
            for (int y = 0; y < height; ++y) {
              std::memcpy(img->data + static_cast<std::size_t>(y) * dst_y_stride,
                          src + static_cast<std::size_t>(y) * src_stride, line_bytes);
            }
          }

          // UV plane
          auto *uv_dst = img->data + img->uv_offset;
          auto *uv_src = src + y_plane_size;
          const auto dst_uv_stride = static_cast<std::size_t>(img->uv_pitch);
          if (src_stride == dst_uv_stride) {
            std::memcpy(uv_dst, uv_src, uv_plane_size);
          } else {
            for (std::size_t y = 0; y < uv_height; ++y) {
              std::memcpy(uv_dst + y * dst_uv_stride, uv_src + y * src_stride, line_bytes);
            }
          }
          return 0;
        }

        const auto dst_stride = static_cast<std::size_t>(img->row_pitch);
        auto *dst_base = img->data;

        // YUV packed formats: use SIMD-accelerated sws_scale.
        if (pixel_format_ == V4L2_PIX_FMT_YUYV || pixel_format_ == V4L2_PIX_FMT_UYVY) {
          if (!sws_) {
            return -1;
          }

          const auto frame_bytes = src_stride * static_cast<std::size_t>(height);
          if (frame_bytes > bytes_used) {
            return -1;
          }

          // YUV → NV12 conversion: output Y plane at dst_base, UV plane at uv_offset
          std::array<const std::uint8_t *, 4> src_data {src, nullptr, nullptr, nullptr};
          std::array<int, 4> src_linesize {static_cast<int>(bytes_per_line_), 0, 0, 0};
          std::array<std::uint8_t *, 4> dst_data {dst_base, dst_base + img->uv_offset, nullptr, nullptr};
          std::array<int, 4> dst_linesize {img->row_pitch, img->uv_pitch, 0, 0};

          if (sws_scale(sws_.get(), src_data.data(), src_linesize.data(), 0, height, dst_data.data(), dst_linesize.data()) != height) {
            return -1;
          }
          return 0;
        }

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

          default:
            BOOST_LOG(error) << "UVC: unsupported pixel format conversion for 0x"sv << util::hex(pixel_format_).to_string_view();
            return -1;
        }
      }

      bool is_supported_pixel_format(std::uint32_t pix_fmt) const {
        switch (pix_fmt) {
          case V4L2_PIX_FMT_NV12:
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
        sws_.reset();

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

      std::uint32_t pixel_format_ {};
      std::uint32_t bytes_per_line_ {};
      bool nv12_passthrough_ {false};
      video::sws_t sws_;
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
