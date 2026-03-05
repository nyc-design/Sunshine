/**
 * @file src/platform/windows/uvc_capture.cpp
 * @brief Windows UVC capture backend implementation.
 */

// standard includes
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string_view>
#include <thread>
#include <vector>

// platform includes
#ifndef NOMINMAX
  #define NOMINMAX
#endif
#include <Windows.h>
#include <ks.h>
#include <ksmedia.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <propvarutil.h>

// local includes
#include "uvc_capture.h"
#include "utf_utils.h"
#include "src/logging.h"
#include "src/utility.h"

using namespace std::literals;

namespace platf::uvc {
  namespace {
    template<class T>
    void release_com(T *ptr) {
      if (ptr) {
        ptr->Release();
      }
    }

    using attributes_t = util::safe_ptr<IMFAttributes, release_com<IMFAttributes>>;
    using activate_t = util::safe_ptr<IMFActivate, release_com<IMFActivate>>;

    std::once_flag mf_startup_once;
    std::atomic_bool mf_startup_ok {false};

    bool ensure_mf_started() {
      std::call_once(mf_startup_once, []() {
        const auto hr = MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: MFStartup failed [0x"sv << util::hex(hr).to_string_view() << ']';
          mf_startup_ok = false;
          return;
        }
        mf_startup_ok = true;
      });

      return mf_startup_ok.load();
    }

    std::string utf8_attr(IMFActivate *activate, const GUID &key) {
      wchar_t *value_w = nullptr;
      UINT32 length = 0;
      const auto hr = activate->GetAllocatedString(key, &value_w, &length);
      if (FAILED(hr) || !value_w) {
        return {};
      }
      (void) length;

      auto release = util::fail_guard([value_w]() {
        CoTaskMemFree(value_w);
      });
      return utf_utils::to_utf8(value_w);
    }
  }  // namespace

  std::vector<device_info_t> enumerate_devices() {
    std::vector<device_info_t> devices;

    const auto co_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    const bool co_initialized = SUCCEEDED(co_hr);
    auto co_guard = util::fail_guard([co_initialized]() {
      if (co_initialized) {
        CoUninitialize();
      }
    });

    if (!ensure_mf_started()) {
      return devices;
    }

    attributes_t attrs;
    auto hr = MFCreateAttributes(&attrs, 1);
    if (FAILED(hr)) {
      BOOST_LOG(error) << "UVC: MFCreateAttributes failed [0x"sv << util::hex(hr).to_string_view() << ']';
      return devices;
    }

    hr = attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    if (FAILED(hr)) {
      BOOST_LOG(error) << "UVC: failed to set video capture source type [0x"sv << util::hex(hr).to_string_view() << ']';
      return devices;
    }

    IMFActivate **raw_devices = nullptr;
    UINT32 count = 0;
    hr = MFEnumDeviceSources(attrs.get(), &raw_devices, &count);
    if (FAILED(hr)) {
      BOOST_LOG(debug) << "UVC: MFEnumDeviceSources failed [0x"sv << util::hex(hr).to_string_view() << ']';
      return devices;
    }

    auto cleanup = util::fail_guard([&]() {
      if (!raw_devices) {
        return;
      }

      for (UINT32 i = 0; i < count; ++i) {
        if (raw_devices[i]) {
          raw_devices[i]->Release();
        }
      }
      CoTaskMemFree(raw_devices);
    });

    devices.reserve(count);
    for (UINT32 i = 0; i < count; ++i) {
      activate_t activate {raw_devices[i]};
      raw_devices[i] = nullptr;

      const auto symbolic_link = utf8_attr(activate.get(), MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK);
      auto friendly_name = utf8_attr(activate.get(), MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME);
      if (friendly_name.empty()) {
        friendly_name = "Capture Device "s + std::to_string(i + 1);
      }

      if (symbolic_link.empty()) {
        BOOST_LOG(debug) << "UVC: skipping capture source without symbolic link: "sv << friendly_name;
        continue;
      }

      device_info_t device;
      device.id = symbolic_link;
      device.device_path = symbolic_link;
      device.display_name = friendly_name;
      devices.emplace_back(std::move(device));
    }

    return devices;
  }
}  // namespace platf::uvc

namespace platf {
  namespace {
    template<class T>
    void release_com(T *ptr) {
      if (ptr) {
        ptr->Release();
      }
    }

    using source_t = util::safe_ptr<IMFMediaSource, release_com<IMFMediaSource>>;
    using reader_t = util::safe_ptr<IMFSourceReader, release_com<IMFSourceReader>>;
    using attributes_t = util::safe_ptr<IMFAttributes, release_com<IMFAttributes>>;
    using media_type_t = util::safe_ptr<IMFMediaType, release_com<IMFMediaType>>;
    using sample_t = util::safe_ptr<IMFSample, release_com<IMFSample>>;
    using buffer_t = util::safe_ptr<IMFMediaBuffer, release_com<IMFMediaBuffer>>;

    struct uvc_img_t: public img_t {
      std::vector<std::uint8_t> backing;
    };

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
        close();
      }

      int init(const uvc::device_info_t &device, const video::config_t &config) {
        source_id_ = device.id;
        source_name_ = device.display_name;

        const auto co_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        if (FAILED(co_hr) && co_hr != RPC_E_CHANGED_MODE) {
          BOOST_LOG(error) << "UVC: CoInitializeEx failed [0x"sv << util::hex(co_hr).to_string_view() << ']';
          return -1;
        }
        co_initialized_ = SUCCEEDED(co_hr);

        if (!uvc::ensure_mf_started()) {
          return -1;
        }

        attributes_t source_attrs;
        auto hr = MFCreateAttributes(&source_attrs, 2);
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: MFCreateAttributes failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        hr = source_attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: failed to set source type [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        auto symbolic_link_w = utf_utils::from_utf8(source_id_);
        hr = source_attrs->SetString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, symbolic_link_w.c_str());
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: failed to set symbolic link [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        source_t source;
        hr = MFCreateDeviceSource(source_attrs.get(), &source);
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: MFCreateDeviceSource failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        attributes_t reader_attrs;
        hr = MFCreateAttributes(&reader_attrs, 2);
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: MFCreateAttributes(reader) failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }
        reader_attrs->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, FALSE);
        reader_attrs->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE);

        hr = MFCreateSourceReaderFromMediaSource(source.get(), reader_attrs.get(), &reader_);
        if (FAILED(hr) || !reader_) {
          BOOST_LOG(error) << "UVC: MFCreateSourceReaderFromMediaSource failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        if (select_media_type(config)) {
          return -1;
        }

        const auto requested_fps = std::max(1, config.framerate);
        delay_ = std::chrono::nanoseconds {1s} / requested_fps;

        BOOST_LOG(info) << "UVC: capturing from "sv << source_name_ << " ["sv << source_id_ << "] "sv << width << 'x' << height;
        return 0;
      }

      capture_e capture(const push_captured_image_cb_t &push_captured_image_cb, const pull_free_image_cb_t &pull_free_image_cb, bool * /*cursor*/) override {
        auto next_frame = std::chrono::steady_clock::now();
        sleep_overshoot_logger.reset();

        while (true) {
          const auto now = std::chrono::steady_clock::now();
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
        return std::make_unique<avcodec_encode_device_t>();
      }

      bool is_codec_supported(std::string_view name, const ::video::config_t & /*config*/) override {
        // The Windows custom NVENC path depends on desktop capture textures. UVC currently routes through avcodec paths.
        return name.find("_nvenc") == std::string_view::npos;
      }

    private:
      int select_media_type(const video::config_t &config) {
        static const std::array preferred_types {
          MFVideoFormat_RGB32,
          MFVideoFormat_YUY2,
          MFVideoFormat_NV12,
        };

        HRESULT hr = S_OK;
        bool selected = false;
        for (const auto &subtype : preferred_types) {
          media_type_t type;
          hr = MFCreateMediaType(&type);
          if (FAILED(hr)) {
            continue;
          }

          type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
          type->SetGUID(MF_MT_SUBTYPE, subtype);

          if (config.width > 0 && config.height > 0) {
            MFSetAttributeSize(type.get(), MF_MT_FRAME_SIZE, static_cast<UINT32>(config.width), static_cast<UINT32>(config.height));
          }
          if (config.framerate > 0) {
            MFSetAttributeRatio(type.get(), MF_MT_FRAME_RATE, static_cast<UINT32>(config.framerate), 1);
          }

          hr = reader_->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, type.get());
          if (SUCCEEDED(hr)) {
            selected = true;
            break;
          }
        }

        if (!selected) {
          BOOST_LOG(debug) << "UVC: no preferred media type accepted; using source default"sv;
        }

        return refresh_current_type();
      }

      int refresh_current_type() {
        media_type_t current_type;
        const auto hr = reader_->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &current_type);
        if (FAILED(hr) || !current_type) {
          BOOST_LOG(error) << "UVC: GetCurrentMediaType failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        UINT32 w = 0;
        UINT32 h = 0;
        if (FAILED(MFGetAttributeSize(current_type.get(), MF_MT_FRAME_SIZE, &w, &h)) || w == 0 || h == 0) {
          BOOST_LOG(error) << "UVC: invalid capture size from media type"sv;
          return -1;
        }

        width = static_cast<int>(w);
        height = static_cast<int>(h);
        env_width = width;
        env_height = height;
        logical_width = width;
        logical_height = height;
        env_logical_width = width;
        env_logical_height = height;

        if (FAILED(current_type->GetGUID(MF_MT_SUBTYPE, &subtype_))) {
          BOOST_LOG(error) << "UVC: missing media subtype"sv;
          return -1;
        }

        if (subtype_ == MFVideoFormat_RGB32) {
          bytes_per_line_ = static_cast<std::uint32_t>(width) * 4;
        } else if (subtype_ == MFVideoFormat_YUY2) {
          bytes_per_line_ = static_cast<std::uint32_t>(width) * 2;
        } else if (subtype_ == MFVideoFormat_NV12) {
          bytes_per_line_ = static_cast<std::uint32_t>(width);
        } else {
          BOOST_LOG(error) << "UVC: unsupported media subtype"sv;
          return -1;
        }

        return 0;
      }

      capture_e capture_frame(img_t *img, std::chrono::milliseconds timeout) {
        if (!img || !img->data || !reader_) {
          return capture_e::error;
        }

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (true) {
          DWORD flags = 0;
          IMFSample *sample_raw = nullptr;
          const auto hr = reader_->ReadSample(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            0,
            nullptr,
            &flags,
            nullptr,
            &sample_raw
          );

          sample_t sample {sample_raw};

          if (FAILED(hr)) {
            BOOST_LOG(error) << "UVC: ReadSample failed [0x"sv << util::hex(hr).to_string_view() << ']';
            return capture_e::reinit;
          }

          if (flags & MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED) {
            if (refresh_current_type()) {
              return capture_e::reinit;
            }
          }

          if (flags & MF_SOURCE_READERF_ENDOFSTREAM) {
            BOOST_LOG(warning) << "UVC: capture stream ended"sv;
            return capture_e::reinit;
          }

          if (sample) {
            if (convert_sample(sample.get(), img)) {
              return capture_e::error;
            }

            img->frame_timestamp = std::chrono::steady_clock::now();
            return capture_e::ok;
          }

          if (std::chrono::steady_clock::now() >= deadline) {
            return capture_e::timeout;
          }

          std::this_thread::sleep_for(1ms);
        }
      }

      int convert_sample(IMFSample *sample, img_t *img) {
        buffer_t buffer;
        const auto hr = sample->ConvertToContiguousBuffer(&buffer);
        if (FAILED(hr) || !buffer) {
          BOOST_LOG(error) << "UVC: ConvertToContiguousBuffer failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        BYTE *data = nullptr;
        DWORD max_len = 0;
        DWORD current_len = 0;
        if (FAILED(buffer->Lock(&data, &max_len, &current_len)) || !data) {
          BOOST_LOG(error) << "UVC: failed to lock media buffer"sv;
          return -1;
        }
        (void) max_len;

        auto unlock = util::fail_guard([&]() {
          buffer->Unlock();
        });

        auto *dst = img->data;
        const auto dst_stride = static_cast<std::size_t>(img->row_pitch);

        if (subtype_ == MFVideoFormat_RGB32) {
          const auto src_stride = static_cast<std::size_t>(bytes_per_line_);
          const auto line_bytes = static_cast<std::size_t>(width) * 4;
          for (int y = 0; y < height; ++y) {
            const auto src_off = static_cast<std::size_t>(y) * src_stride;
            const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
            if (src_off + line_bytes > current_len) {
              return -1;
            }
            std::memcpy(dst + dst_off, data + src_off, line_bytes);
          }
          return 0;
        }

        if (subtype_ == MFVideoFormat_YUY2) {
          const auto src_stride = static_cast<std::size_t>(bytes_per_line_);
          for (int y = 0; y < height; ++y) {
            const auto src_off = static_cast<std::size_t>(y) * src_stride;
            const auto dst_off = static_cast<std::size_t>(y) * dst_stride;
            if (src_off + src_stride > current_len) {
              return -1;
            }

            const auto *line = data + src_off;
            auto *out = dst + dst_off;
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

        if (subtype_ == MFVideoFormat_NV12) {
          const auto y_stride = static_cast<std::size_t>(bytes_per_line_);
          const auto uv_stride = static_cast<std::size_t>(bytes_per_line_);
          const auto y_plane_size = y_stride * static_cast<std::size_t>(height);
          if (current_len < y_plane_size) {
            return -1;
          }

          const auto *y_plane = data;
          const auto *uv_plane = data + y_plane_size;

          for (int y = 0; y < height; ++y) {
            auto *out = dst + static_cast<std::size_t>(y) * dst_stride;
            const auto y_row_off = static_cast<std::size_t>(y) * y_stride;
            const auto uv_row_off = static_cast<std::size_t>(y / 2) * uv_stride;

            for (int x = 0; x < width; ++x) {
              const int yy = y_plane[y_row_off + static_cast<std::size_t>(x)];
              const auto uv_off = uv_row_off + static_cast<std::size_t>(x / 2) * 2;
              const int u = uv_plane[uv_off + 0];
              const int v = uv_plane[uv_off + 1];
              yuv_to_bgr0(yy, u, v, out + static_cast<std::size_t>(x) * 4);
            }
          }
          return 0;
        }

        return -1;
      }

      void close() {
        reader_.reset();

        if (co_initialized_) {
          CoUninitialize();
          co_initialized_ = false;
        }
      }

      std::string source_id_;
      std::string source_name_;

      bool co_initialized_ {false};
      reader_t reader_;
      GUID subtype_ {MFVideoFormat_RGB32};
      std::uint32_t bytes_per_line_ {0};
      std::chrono::nanoseconds delay_ {16666666ns};
    };

    const uvc::device_info_t *find_device(std::string_view display_name, const std::vector<uvc::device_info_t> &devices) {
      for (const auto &device : devices) {
        if (display_name.empty() || display_name == device.id || display_name == device.device_path || display_name == device.display_name) {
          return &device;
        }
      }

      return nullptr;
    }
  }  // namespace

  std::vector<std::string> uvc_display_names() {
    std::vector<std::string> names;
    for (const auto &device : uvc::enumerate_devices()) {
      names.emplace_back(device.id);
    }
    return names;
  }

  std::shared_ptr<display_t> uvc_display(mem_type_e hwdevice_type, const std::string &display_name, const video::config_t &config) {
    if (hwdevice_type != mem_type_e::system && hwdevice_type != mem_type_e::dxgi) {
      BOOST_LOG(error) << "UVC: unsupported hw device type requested"sv;
      return nullptr;
    }

    auto devices = uvc::enumerate_devices();
    if (devices.empty()) {
      BOOST_LOG(error) << "UVC: no capture devices detected"sv;
      return nullptr;
    }

    const auto *device = find_device(display_name, devices);
    if (!device) {
      BOOST_LOG(error) << "UVC: requested source not found: "sv << display_name;
      return nullptr;
    }

    auto display = std::make_shared<uvc_display_t>();
    if (display->init(*device, config)) {
      return nullptr;
    }

    return display;
  }
}  // namespace platf
