// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#pragma once

#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "thirdparty/junction/LogTable.h"

/**
 * Converts log tables to the RLOG format. Based on RLOG R2 with
 * support for custom type strings.
 */
class RLOGEncoder {
public:
  static const uint8_t logRevision = 2;

  RLOGEncoder();

  /** Reads the encoded output of the last encoded table. */
  std::vector<uint8_t> GetOutput();

  /**
   * Returns data required to start a new receiver (full contents of last table
   * + all key IDs).
   */
  std::vector<uint8_t> GetNewcomerData();

  /** Encodes a single table and stores the result. */
  void EncodeTable(LogTable &table, bool includeRevision);

private:
  std::vector<uint8_t> EncodeTimestamp(double timestamp);
  std::vector<uint8_t> EncodeKey(uint16_t keyID, const std::string &key,
                                 const std::string &type);
  std::vector<uint8_t> EncodeValue(uint16_t keyID,
                                   const LogTable::LogValue &value);

  std::vector<uint8_t> nextOutput;
  bool isFirstTable = true;
  LogTable lastTable = LogTable(0);
  std::map<std::string, uint16_t> keyIDs;
  std::map<std::string, std::string> keyTypes;
  uint16_t nextKeyID = 0;
};