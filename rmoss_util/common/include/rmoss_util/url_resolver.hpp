// Copyright 2023 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This program is based on https://github.com/ros-perception/image_common
// which are released under the BSD License.
// https://opensource.org/license/bsd-3-clause/

#ifndef RMOSS_UTIL__URL_RESOLVER_HPP_
#define RMOSS_UTIL__URL_RESOLVER_HPP_

#include <string>

namespace rmoss_util
{
class URLResolver
{
public:
  /**
   * @brief Seperate url
   *
   * @param url
   * @return std::string
   */
  static std::string resolve_url(const std::string & url);

  /**
   * @brief Get the Resolved file path
   *
   * @param url
   * @return std::string Return empty string if resolve failed
   */
  static std::string get_resolved_path(const std::string & url);

private:
  typedef enum
  {
    // supported URLs
    URL_empty = 0,  // empty string
    URL_file,       // file:
    URL_package,    // package:
                    // URLs not supported
    URL_invalid,    // anything >= is invalid
  } url_type_t;

  static url_type_t parse_url(const std::string & url);

  static std::string get_package_fileName(const std::string & url);
};
}  // namespace rmoss_util

#endif  // RMOSS_UTIL__URL_RESOLVER_HPP_
