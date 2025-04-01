// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/LoggedTunableNumber.h"

#include "Constants.h"

namespace util {

constexpr char LoggedTunableNumber::tableKey[];

LoggedTunableNumber::LoggedTunableNumber(const std::string &dashboardKey)
    : key(std::string(tableKey) + "/" + dashboardKey) {}

LoggedTunableNumber::LoggedTunableNumber(const std::string &dashboardKey,
                                         double defaultValue)
    : LoggedTunableNumber(dashboardKey) {
  InitDefault(defaultValue);
}

void LoggedTunableNumber::InitDefault(double defaultValue) {
  if (!hasDefault) {
    hasDefault = true;
    this->defaultValue = defaultValue;
    if (Constants::tuningMode && !Constants::disableHAL) {
      dashboardNumber =
          std::make_unique<wpi::log::DataLogEntry>(key, defaultValue);
    }
  }
}

double LoggedTunableNumber::Get() const {
  if (!hasDefault) {
    return 0.0;
  } else {
    return Constants::tuningMode && !Constants::disableHAL
               ? dashboardNumber->GetDouble(defaultValue)
               : defaultValue;
  }
}

bool LoggedTunableNumber::HasChanged(int id) {
  double currentValue = Get();
  auto it = lastHasChangedValues.find(id);
  if (it == lastHasChangedValues.end() || currentValue != it->second) {
    lastHasChangedValues[id] = currentValue;
    return true;
  }
  return false;
}

void LoggedTunableNumber::IfChanged(
    int id, std::function<void(const std::vector<double> &)> action,
    const std::vector<LoggedTunableNumber *> &tunableNumbers) {
  if (std::any_of(
          tunableNumbers.begin(), tunableNumbers.end(),
          [id](LoggedTunableNumber *num) { return num->HasChanged(id); })) {
    std::vector<double> values;
    for (LoggedTunableNumber *num : tunableNumbers) {
      values.push_back(num->Get());
    }
    action(values);
  }
}

void LoggedTunableNumber::IfChanged(
    int id, std::function<void()> action,
    const std::vector<LoggedTunableNumber *> &tunableNumbers) {
  IfChanged(
      id, [action](const std::vector<double> &) { action(); }, tunableNumbers);
}

double LoggedTunableNumber::operator()() const { return Get(); }

} // namespace util