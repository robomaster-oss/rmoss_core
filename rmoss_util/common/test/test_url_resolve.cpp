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

#include <string>

#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rmoss_util/url_resolver.hpp"

TEST(URLResolver, package)
{
  std::string url = "package://rmoss_util/test";
  std::string gt_path = ament_index_cpp::get_package_share_directory("rmoss_util") + "/test";

  EXPECT_STREQ(rmoss_util::URLResolver::get_resolved_path(url).c_str(), gt_path.c_str());
}

TEST(URLResolver, file)
{
  std::string url = "file:///test_dir/test_file";
  std::string gt_path = "/test_dir/test_file";

  EXPECT_STREQ(rmoss_util::URLResolver::get_resolved_path(url).c_str(), gt_path.c_str());
}
