/**
 * @file src/platform/windows/uvc_capture.cpp
 * @brief Windows UVC capture backend implementation.
 */

// standard includes
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
#include "src/video.h"

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

    const char *subtype_name(const GUID &subtype) {
      if (subtype == MFVideoFormat_RGB32) {
        return "RGB32";
      }
      if (subtype == MFVideoFormat_YUY2) {
        return "YUY2";
      }
      if (subtype == MFVideoFormat_NV12) {
        return "NV12";
      }
      return "unknown";
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
        hr = MFCreateAttributes(&reader_attrs, 3);
        if (FAILED(hr)) {
          BOOST_LOG(error) << "UVC: MFCreateAttributes(reader) failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }
        reader_attrs->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, TRUE);
        reader_attrs->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, FALSE);
#ifdef MF_LOW_LATENCY
        reader_attrs->SetUINT32(MF_LOW_LATENCY, TRUE);
#endif

        hr = MFCreateSourceReaderFromMediaSource(source.get(), reader_attrs.get(), &reader_);
        if (FAILED(hr) || !reader_) {
          BOOST_LOG(error) << "UVC: MFCreateSourceReaderFromMediaSource failed [0x"sv << util::hex(hr).to_string_view() << ']';
          return -1;
        }

        if (select_media_type(config)) {
          return -1;
        }

        const auto requested_fps = std::max(1, config.framerate);
        const auto nominal_frame = std::chrono::milliseconds {std::max(1, 1000 / requested_fps)};
        read_timeout_ = std::clamp(nominal_frame * 6, 100ms, 2000ms);

        BOOST_LOG(info) << "UVC: capturing from "sv << source_name_ << " ["sv << source_id_ << "] "sv << width << 'x' << height;
        return 0;
      }

      capture_e capture(const push_captured_image_cb_t &push_captured_image_cb, const pull_free_image_cb_t &pull_free_image_cb, bool * /*cursor*/) override {
        // Back-pressure: track the last pushed frame so we can wait for the
        // encoder to consume it before capturing the next one. Without this,
        // the video pipeline's single-value event latch silently overwrites
        // unconsumed frames, causing periodic stuttering (especially during
        // IDR keyframe encoding which takes longer). Media Foundation's
        // internal buffer queues incoming UVC frames while we wait.
        std::shared_ptr<img_t> last_pushed;

        while (true) {
          // Wait for the encoder to finish with the previous frame.
          // use_count == 2 means only the pool and our ref remain (encoder is done).
          if (last_pushed) {
            auto wait_start = std::chrono::steady_clock::now();
            while (last_pushed.use_count() > 2) {
              if (std::chrono::steady_clock::now() - wait_start > read_timeout_) {
                break;  // don't stall forever
              }
              SwitchToThread();
            }
            last_pushed.reset();
          }

          std::shared_ptr<img_t> img_out;
          if (!pull_free_image_cb(img_out)) {
            return capture_e::interrupted;
          }

          auto status = capture_frame(img_out.get(), read_timeout_);
          switch (status) {
            case capture_e::ok:
              last_pushed = img_out;  // keep ref for back-pressure check
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
          // NV12: Y plane (width * height) + UV plane (width * height/2)
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

        std::size_t bytes;
        if (img->source_format == img_t::nv12) {
          bytes = static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height) +
                  static_cast<std::size_t>(img->uv_pitch) * static_cast<std::size_t>((img->height + 1) / 2);
          // NV12 black: Y=16 (limited range black), UV=128 (neutral chroma)
          const auto y_size = static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height);
          std::fill_n(img->data, y_size, static_cast<std::uint8_t>(16));
          std::fill_n(img->data + img->uv_offset, bytes - y_size, static_cast<std::uint8_t>(128));
        } else {
          bytes = static_cast<std::size_t>(img->row_pitch) * static_cast<std::size_t>(img->height);
          std::fill_n(img->data, bytes, 0);
        }

        img->frame_timestamp = std::chrono::steady_clock::now();
        return 0;
      }

      std::unique_ptr<avcodec_encode_device_t> make_avcodec_encode_device(pix_fmt_e /*pix_fmt*/) override {
        // Return base class with null data — video.cpp will create avcodec_software_encode_device_t
        // which handles BGR0→NV12 conversion and av_hwframe_transfer_data for hw encoders.
        return std::make_unique<avcodec_encode_device_t>();
      }

      bool is_codec_supported(std::string_view /*name*/, const ::video::config_t & /*config*/) override {
        // Allow all codecs including NVENC through the avcodec software upload path.
        // The custom NVENC texture path won't be used (we don't override make_nvenc_encode_device),
        // but h264_nvenc/hevc_nvenc via FFmpeg will work through avcodec_software_encode_device_t
        // with av_hwframe_transfer_data for GPU upload.
        return true;
      }

    private:
      int select_media_type(const video::config_t &config) {
        // NV12 first: it's the encoder's native format, avoiding double color conversion.
        // YUY2 second: common capture card format, converted via sws_scale.
        // RGB32 last: fallback, direct memcpy but requires encoder-side conversion.
        static const std::array preferred_types {
          MFVideoFormat_NV12,
          MFVideoFormat_YUY2,
          MFVideoFormat_RGB32,
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

        UINT32 raw_stride = 0;
        int default_stride = 0;
        if (SUCCEEDED(current_type->GetUINT32(MF_MT_DEFAULT_STRIDE, &raw_stride))) {
          default_stride = static_cast<int>(raw_stride);
        }

        // NV12 passthrough: deliver NV12 data to encoder, avoiding double conversion.
        // Both native NV12 and YUY2 (converted to NV12) use this path.
        nv12_passthrough_ = (subtype_ == MFVideoFormat_NV12 || subtype_ == MFVideoFormat_YUY2);

        if (subtype_ == MFVideoFormat_NV12) {
          if (default_stride == 0) {
            default_stride = width;
          }
          sws_.reset();
        } else if (subtype_ == MFVideoFormat_RGB32) {
          if (default_stride == 0) {
            default_stride = width * 4;
          }
          sws_.reset();
        } else if (subtype_ == MFVideoFormat_YUY2) {
          if (default_stride == 0) {
            default_stride = width * 2;
          }
          // Convert YUY2 → NV12 directly (single conversion) instead of
          // YUY2 → BGR0 → NV12 (double conversion that caused stuttering).
          video::sws_t next_sws {sws_getContext(
            width, height, AV_PIX_FMT_YUYV422,
            width, height, AV_PIX_FMT_NV12,
            SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
          )};
          if (!next_sws) {
            BOOST_LOG(error) << "UVC: failed to initialize YUY2→NV12 conversion context"sv;
            return -1;
          }
          sws_ = std::move(next_sws);
        } else {
          BOOST_LOG(error) << "UVC: unsupported media subtype"sv;
          return -1;
        }

        bytes_per_line_ = static_cast<std::uint32_t>(std::abs(default_stride));

        UINT32 frame_rate_num = 0;
        UINT32 frame_rate_den = 0;
        if (SUCCEEDED(MFGetAttributeRatio(current_type.get(), MF_MT_FRAME_RATE, &frame_rate_num, &frame_rate_den)) &&
            frame_rate_num > 0 && frame_rate_den > 0) {
          const auto fps = static_cast<double>(frame_rate_num) / static_cast<double>(frame_rate_den);
          if (fps > 0.0) {
            read_timeout_ = std::clamp(
              std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double> {6.0 / fps}),
              100ms,
              2000ms
            );
            BOOST_LOG(info) << "UVC: negotiated format "sv << subtype_name(subtype_) << ' ' << width << 'x' << height << '@'
                            << fps << "fps (passthrough=" << (nv12_passthrough_ ? "yes" : "no") << ')';
          }
        } else {
          BOOST_LOG(info) << "UVC: negotiated format "sv << subtype_name(subtype_) << ' ' << width << 'x' << height
                          << " (passthrough=" << (nv12_passthrough_ ? "yes" : "no") << ')';
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

          // ReadSample is already blocking; yield briefly only if it returned no sample.
          SwitchToThread();
        }
      }

      int convert_sample(IMFSample *sample, img_t *img) {
        // Try to get the first buffer directly to avoid a potential copy in ConvertToContiguousBuffer.
        DWORD buffer_count = 0;
        sample->GetBufferCount(&buffer_count);

        buffer_t buffer;
        HRESULT hr;
        if (buffer_count == 1) {
          hr = sample->GetBufferByIndex(0, &buffer);
        } else {
          hr = sample->ConvertToContiguousBuffer(&buffer);
        }
        if (FAILED(hr) || !buffer) {
          BOOST_LOG(error) << "UVC: failed to get media buffer [0x"sv << util::hex(hr).to_string_view() << ']';
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

        // NV12 native: copy planes directly without color conversion.
        if (subtype_ == MFVideoFormat_NV12) {
          const auto src_stride = static_cast<std::size_t>(bytes_per_line_);
          const auto y_plane_size = src_stride * static_cast<std::size_t>(height);
          const auto uv_height = static_cast<std::size_t>((height + 1) / 2);
          const auto uv_plane_size = src_stride * uv_height;
          if (current_len < y_plane_size + uv_plane_size) {
            return -1;
          }

          const auto dst_y_stride = static_cast<std::size_t>(img->row_pitch);
          const auto line_bytes = static_cast<std::size_t>(width);

          // Copy Y plane
          if (src_stride == dst_y_stride) {
            std::memcpy(dst, data, y_plane_size);
          } else {
            for (int y = 0; y < height; ++y) {
              std::memcpy(dst + static_cast<std::size_t>(y) * dst_y_stride,
                          data + static_cast<std::size_t>(y) * src_stride,
                          line_bytes);
            }
          }

          // Copy UV plane
          auto *uv_dst = dst + img->uv_offset;
          auto *uv_src = data + y_plane_size;
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
          if (!sws_) {
            return -1;
          }

          const auto src_stride = static_cast<int>(bytes_per_line_);
          const auto frame_bytes = static_cast<std::size_t>(src_stride) * static_cast<std::size_t>(height);
          if (frame_bytes > current_len) {
            return -1;
          }

          // YUY2 → NV12 conversion: output Y plane at dst, UV plane at uv_offset
          std::array<std::uint8_t *, 4> src_data {data, nullptr, nullptr, nullptr};
          std::array<int, 4> src_linesize {src_stride, 0, 0, 0};
          std::array<std::uint8_t *, 4> dst_data {dst, dst + img->uv_offset, nullptr, nullptr};
          std::array<int, 4> dst_linesize {img->row_pitch, img->uv_pitch, 0, 0};

          if (sws_scale(sws_.get(), src_data.data(), src_linesize.data(), 0, height, dst_data.data(), dst_linesize.data()) != height) {
            return -1;
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
      GUID subtype_ {MFVideoFormat_NV12};
      std::uint32_t bytes_per_line_ {0};
      std::chrono::milliseconds read_timeout_ {250ms};
      video::sws_t sws_;
      bool nv12_passthrough_ {false};
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
