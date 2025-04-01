// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/NTClientLogger.h"

#include <wpi/Logger.h>

namespace util {

constexpr char NTClientLogger::tableName[];
std::set<std::string> NTClientLogger::lastRemoteIds;
std::vector<uint8_t> NTClientLogger::intBuffer(4);

void NTClientLogger::Log() {
  auto connections = nt::NetworkTableInstance::GetDefault().GetConnections();
  std::set<std::string> remoteIds;

  // Log data for connected clients
  for (const auto &connection : connections) {
    lastRemoteIds.erase(connection.remoteId);
    remoteIds.insert(connection.remoteId);
    wpi::log::DataLog::RecordOutput(
        std::string(tableName) + connection.remoteId + "/Connected", true);
    wpi::log::DataLog::RecordOutput(std::string(tableName) +
                                        connection.remoteId + "/IPAddress",
                                    connection.remoteIp);
    wpi::log::DataLog::RecordOutput(std::string(tableName) +
                                        connection.remoteId + "/RemotePort",
                                    static_cast<double>(connection.remotePort));
    wpi::log::DataLog::RecordOutput(std::string(tableName) +
                                        connection.remoteId + "/LastUpdate",
                                    connection.lastUpdate);

    int32_t protocolVersion = connection.protocolVersion;
    for (size_t i = 0; i < 4; ++i) {
      intBuffer[i] = static_cast<uint8_t>((protocolVersion >> (i * 8)) & 0xFF);
    }
    wpi::log::DataLog::RecordOutput(
        std::string(tableName) + connection.remoteId + "/ProtocolVersion",
        intBuffer);
  }

  // Mark disconnected clients
  for (const auto &remoteId : lastRemoteIds) {
    wpi::log::DataLog::RecordOutput(
        std::string(tableName) + remoteId + "/Connected", false);
  }
  lastRemoteIds = remoteIds;
}

} // namespace util