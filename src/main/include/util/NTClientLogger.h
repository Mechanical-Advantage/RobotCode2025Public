// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <set>
#include <string>
#include <vector>

#include <networktables/ConnectionInfo.h>
#include <networktables/NetworkTableInstance.h>

namespace util {

/** Utility class to log the list of NetworkTables clients. */
class NTClientLogger {
public:
  static void Log();

private:
  NTClientLogger() = delete; // Prevent instantiation
  static constexpr char tableName[] = "NTClients/";
  static std::set<std::string> lastRemoteIds;
  static std::vector<uint8_t> intBuffer;
};

} // namespace util