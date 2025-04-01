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

#include "util/rlog/RLOGEncoder.h"

#include <cstring>
#include <sstream>

RLOGEncoder::RLOGEncoder() : lastTable(0), nextKeyID(0), isFirstTable(true) {}

std::vector<uint8_t> RLOGEncoder::GetOutput() { return nextOutput; }

std::vector<uint8_t> RLOGEncoder::GetNewcomerData() {
  std::vector<std::vector<uint8_t>> buffers;

  // Encode log revision
  std::vector<uint8_t> revisionBuffer(1);
  revisionBuffer[0] = logRevision;
  buffers.push_back(revisionBuffer);

  // Encode timestamp
  buffers.push_back(EncodeTimestamp(lastTable.GetTimestamp() / 1000000.0));

  // Encode key IDs
  for (const auto &keyID : keyIDs) {
    buffers.push_back(
        EncodeKey(keyID.second, keyID.first, keyTypes[keyID.first]));
  }

  // Encode fields
  for (const auto &field : lastTable.GetAll(false)) {
    buffers.push_back(EncodeValue(keyIDs[field.first], field.second));
  }

  // Combine buffers
  size_t capacity = 0;
  for (const auto &buffer : buffers) {
    capacity += buffer.size();
  }
  std::vector<uint8_t> output(capacity);
  size_t offset = 0;
  for (const auto &buffer : buffers) {
    std::memcpy(output.data() + offset, buffer.data(), buffer.size());
    offset += buffer.size();
  }
  return output;
}

void RLOGEncoder::EncodeTable(LogTable &table, bool includeRevision) {
  std::vector<std::vector<uint8_t>> buffers;

  std::map<std::string, LogTable::LogValue> newMap = table.GetAll(false);
  std::map<std::string, LogTable::LogValue> oldMap = lastTable.GetAll(false);

  // Encode log revision
  if (isFirstTable && includeRevision) {
    std::vector<uint8_t> revisionBuffer(1);
    revisionBuffer[0] = logRevision;
    buffers.push_back(revisionBuffer);
    isFirstTable = false;
  }

  // Encode timestamp
  buffers.push_back(EncodeTimestamp(table.GetTimestamp() / 1000000.0));

  // Encode new/changed fields
  for (const auto &field : newMap) {
    // Check if field has changed
    LogTable::LogValue newValue = field.second;
    if (newValue.Equals(oldMap[field.first])) {
      continue;
    }

    // Write new data
    if (keyIDs.find(field.first) == keyIDs.end()) {
      keyIDs[field.first] = nextKeyID;
      keyTypes[field.first] = field.second.GetWPILOGType();
      buffers.push_back(
          EncodeKey(nextKeyID, field.first, field.second.GetWPILOGType()));
      nextKeyID++;
    }
    buffers.push_back(EncodeValue(keyIDs[field.first], newValue));
  }

  // Update last table
  lastTable = table;

  // Combine buffers
  size_t capacity = 0;
  for (const auto &buffer : buffers) {
    capacity += buffer.size();
  }
  nextOutput.resize(capacity);
  size_t offset = 0;
  for (const auto &buffer : buffers) {
    std::memcpy(nextOutput.data() + offset, buffer.data(), buffer.size());
    offset += buffer.size();
  }
}

std::vector<uint8_t> RLOGEncoder::EncodeTimestamp(double timestamp) {
  std::vector<uint8_t> buffer(1 + sizeof(double));
  buffer[0] = 0;
  std::memcpy(buffer.data() + 1, &timestamp, sizeof(double));
  return buffer;
}

std::vector<uint8_t> RLOGEncoder::EncodeKey(uint16_t keyID,
                                            const std::string &key,
                                            const std::string &type) {
  std::vector<uint8_t> keyBytes(key.begin(), key.end());
  std::vector<uint8_t> typeBytes(type.begin(), type.end());
  std::vector<uint8_t> buffer(1 + sizeof(uint16_t) + sizeof(uint16_t) +
                              keyBytes.size() + sizeof(uint16_t) +
                              typeBytes.size());
  buffer[0] = 1;
  std::memcpy(buffer.data() + 1, &keyID, sizeof(uint16_t));
  uint16_t keyLength = keyBytes.size();
  std::memcpy(buffer.data() + 1 + sizeof(uint16_t), &keyLength,
              sizeof(uint16_t));
  std::memcpy(buffer.data() + 1 + 2 * sizeof(uint16_t), keyBytes.data(),
              keyBytes.size());
  uint16_t typeLength = typeBytes.size();
  std::memcpy(buffer.data() + 1 + 2 * sizeof(uint16_t) + keyBytes.size(),
              &typeLength, sizeof(uint16_t));
  std::memcpy(buffer.data() + 1 + 3 * sizeof(uint16_t) + keyBytes.size(),
              typeBytes.data(), typeBytes.size());
  return buffer;
}

std::vector<uint8_t> RLOGEncoder::EncodeValue(uint16_t keyID,
                                              const LogTable::LogValue &value) {
  std::vector<uint8_t> keyBuffer(1 + sizeof(uint16_t) + sizeof(uint16_t));
  keyBuffer[0] = 2;
  std::memcpy(keyBuffer.data() + 1, &keyID, sizeof(uint16_t));

  std::vector<uint8_t> valueBuffer;
  switch (value.type) {
  case LogTable::LogValue::Type::Raw: {
    const std::vector<uint8_t> &byteArray = value.GetRaw();
    valueBuffer = byteArray;
    break;
  }
  case LogTable::LogValue::Type::Boolean: {
    valueBuffer.resize(1);
    valueBuffer[0] = value.GetBoolean() ? 1 : 0;
    break;
  }
  case LogTable::LogValue::Type::Integer: {
    valueBuffer.resize(sizeof(int64_t));
    int64_t intValue = value.GetInteger();
    std::memcpy(valueBuffer.data(), &intValue, sizeof(int64_t));
    break;
  }
  case LogTable::LogValue::Type::Float: {
    valueBuffer.resize(sizeof(float));
    float floatValue = value.GetFloat();
    std::memcpy(valueBuffer.data(), &floatValue, sizeof(float));
    break;
  }
  case LogTable::LogValue::Type::Double: {
    valueBuffer.resize(sizeof(double));
    double doubleValue = value.GetDouble();
    std::memcpy(valueBuffer.data(), &doubleValue, sizeof(double));
    break;
  }
  case LogTable::LogValue::Type::String: {
    const std::string &stringValue = value.GetString();
    valueBuffer.assign(stringValue.begin(), stringValue.end());
    break;
  }
  case LogTable::LogValue::Type::BooleanArray: {
    const std::vector<bool> &booleanArray = value.GetBooleanArray();
    valueBuffer.resize(booleanArray.size());
    for (size_t i = 0; i < booleanArray.size(); ++i) {
      valueBuffer[i] = booleanArray[i] ? 1 : 0;
    }
    break;
  }
  case LogTable::LogValue::Type::IntegerArray: {
    const std::vector<int64_t> &intArray = value.GetIntegerArray();
    valueBuffer.resize(intArray.size() * sizeof(int64_t));
    for (size_t i = 0; i < intArray.size(); ++i) {
      std::memcpy(valueBuffer.data() + i * sizeof(int64_t), &intArray[i],
                  sizeof(int64_t));
    }
    break;
  }
  case LogTable::LogValue::Type::FloatArray: {
    const std::vector<float> &floatArray = value.GetFloatArray();
    valueBuffer.resize(floatArray.size() * sizeof(float));
    for (size_t i = 0; i < floatArray.size(); ++i) {
      std::memcpy(valueBuffer.data() + i * sizeof(float), &floatArray[i],
                  sizeof(float));
    }
    break;
  }
  case LogTable::LogValue::Type::DoubleArray: {
    const std::vector<double> &doubleArray = value.GetDoubleArray();
    valueBuffer.resize(doubleArray.size() * sizeof(double));
    for (size_t i = 0; i < doubleArray.size(); ++i) {
      std::memcpy(valueBuffer.data() + i * sizeof(double), &doubleArray[i],
                  sizeof(double));
    }
    break;
  }
  case LogTable::LogValue::Type::StringArray: {
    const std::vector<std::string> &stringArray = value.GetStringArray();
    size_t capacity = sizeof(int32_t);
    for (const std::string &i : stringArray) {
      capacity += sizeof(int32_t) + i.size();
    }
    valueBuffer.resize(capacity);
    int32_t stringArraySize = stringArray.size();
    std::memcpy(valueBuffer.data(), &stringArraySize, sizeof(int32_t));
    size_t offset = sizeof(int32_t);
    for (const std::string &i : stringArray) {
      int32_t stringLength = i.size();
      std::memcpy(valueBuffer.data() + offset, &stringLength, sizeof(int32_t));
      offset += sizeof(int32_t);
      std::memcpy(valueBuffer.data() + offset, i.data(), i.size());
      offset += i.size();
    }
    break;
  }
  default:
    break;
  }

  uint16_t valueLength = valueBuffer.size();
  std::memcpy(keyBuffer.data() + 1 + sizeof(uint16_t), &valueLength,
              sizeof(uint16_t));
  std::vector<uint8_t> result(keyBuffer.size() + valueBuffer.size());
  std::memcpy(result.data(), keyBuffer.data(), keyBuffer.size());
  std::memcpy(result.data() + keyBuffer.size(), valueBuffer.data(),
              valueBuffer.size());
  return result;
}
