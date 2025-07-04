// Copyright 2025 The Autoware Contributors
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

#ifndef COMMON__UTILS__LOGGER_HPP_
#define COMMON__UTILS__LOGGER_HPP_

#include <string>

namespace autoware::diagnostic_graph_aggregator
{

class Logger
{
public:
  virtual ~Logger() = default;
  virtual void info(const std::string & message) = 0;
  virtual void debug(const std::string & message) = 0;
};

class DummyLogger : public Logger
{
public:
  void info(const std::string &) override {}
  void debug(const std::string &) override {}
};

class StdLogger : public Logger
{
public:
  void info(const std::string & message) override;
  void debug(const std::string & message) override;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__UTILS__LOGGER_HPP_
