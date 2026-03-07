/**
 * @file src/platform/linux/uvcgrab.h
 * @brief Declarations for Linux UVC capture card support.
 */
#pragma once

// standard includes
#include <string>
#include <vector>

// local includes
#include "src/platform/common.h"

namespace platf::uvc {
  struct device_info_t {
    std::string id;           ///< Stable identifier used by Sunshine (prefer /dev/v4l/by-id path).
    std::string display_name; ///< Human-readable name for UI/generated apps.
    std::string device_path;  ///< Resolved /dev/video* path.
  };

  /**
   * @brief Enumerate UVC capture devices.
   * @return Vector of detected UVC device descriptors.
   */
  std::vector<device_info_t> enumerate_devices();
}  // namespace platf::uvc

namespace platf {
  std::vector<std::string> uvc_display_names();
  std::shared_ptr<display_t> uvc_display(mem_type_e hwdevice_type, const std::string &display_name, const video::config_t &config);
}  // namespace platf
