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

#include "rmoss_util/url_resolver.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/env.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace rmoss_util
{

std::string URLResolver::get_resolved_path(const std::string & url)
{
  const std::string resURL(resolve_url(url));
  url_type_t url_type = parse_url(url);

  std::string res = "";

  switch (url_type) {
    case URL_empty:
      {
        break;
      }
    case URL_file:
      {
        res = resURL.substr(7);
        break;
      }
    case URL_package:
      {
        res = get_package_fileName(resURL);
        break;
      }
    default:
      {
        break;
      }
  }

  return res;
}

std::string URLResolver::resolve_url(const std::string & url)
{
  std::string resolved;
  size_t rest = 0;

  while (true) {
    // find the next '$' in the URL string
    size_t dollar = url.find('$', rest);

    if (dollar >= url.length()) {
      // no more variables left in the URL
      resolved += url.substr(rest);
      break;
    }

    // copy characters up to the next '$'
    resolved += url.substr(rest, dollar - rest);

    if (url.substr(dollar + 1, 1) != "{") {
      // no '{' follows, so keep the '$'
      resolved += "$";
    } else if (url.substr(dollar + 1, 10) == "{ROS_HOME}") {
      // substitute $ROS_HOME
      std::string ros_home;
      std::string ros_home_env = rcpputils::get_env_var("ROS_HOME");
      std::string home_env = rcpputils::get_env_var("HOME");
      if (!ros_home_env.empty()) {
        // use environment variable
        ros_home = ros_home_env;
      } else if (!home_env.empty()) {
        // use "$HOME/.ros"
        ros_home = home_env;
        ros_home += "/.ros";
      }
      resolved += ros_home;
      dollar += 10;
    } else {
      // not a valid substitution variable
      resolved += "$";                // keep the bogus '$'
    }

    // look for next '$'
    rest = dollar + 1;
  }

  return resolved;
}

URLResolver::url_type_t URLResolver::parse_url(const std::string & url)
{
  if (url == "") {
    return URL_empty;
  }

  // Easy C++14 replacement for boost::iequals from :
  // https://stackoverflow.com/a/4119881
  auto iequals = [](const std::string & a, const std::string & b) {
      return std::equal(
        a.begin(), a.end(),
        b.begin(), b.end(),
        [](char a, char b) {
          return tolower(a) == tolower(b);
        });
    };

  if (iequals(url.substr(0, 8), "file:///")) {
    return URL_file;
  }
  if (iequals(url.substr(0, 10), "package://")) {
    // look for a '/' following the package name, make sure it is
    // there, the name is not empty, and something follows it
    size_t rest = url.find('/', 10);
    if (rest < url.length() - 1 && rest > 10) {
      return URL_package;
    }
  }
  return URL_invalid;
}

std::string URLResolver::get_package_fileName(const std::string & url)
{
  // Scan URL from after "package://" until next '/' and extract
  // package name.  The parseURL() already checked that it's present.
  size_t prefix_len = std::string("package://").length();
  size_t rest = url.find('/', prefix_len);
  std::string package(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  std::string pkgPath = ament_index_cpp::get_package_share_directory(package);
  if (pkgPath.empty()) {  // package not found?
    return pkgPath;
  } else {
    // Construct file name from package location and remainder of URL.
    return pkgPath + url.substr(rest);
  }
}

}  // namespace rmoss_util
