/**
 * @file src/platform/windows/uvc_capture.h
 * @brief Declarations for Windows UVC capture card support.
 */
#pragma once

// standard includes
#include <string>
#include <vector>

// local includes
#include "src/platform/common.h"

namespace platf::uvc {
  struct device_info_t {
    std::string id;  ///< Stable identifier (Media Foundation symbolic link).
    std::string display_name;  ///< Human-readable display name.
    std::string device_path;  ///< Underlying symbolic link (same as id on Windows).
  };

  /**
   * @brief Enumerate UVC-style video capture devices via Media Foundation.
   * @return Vector of detected capture device descriptors.
   */
  std::vector<device_info_t> enumerate_devices();
}  // namespace platf::uvc

namespace platf {
  std::vector<std::string> uvc_display_names();
  std::shared_ptr<display_t> uvc_display(mem_type_e hwdevice_type, const std::string &display_name, const video::config_t &config);
}  // namespace platf
