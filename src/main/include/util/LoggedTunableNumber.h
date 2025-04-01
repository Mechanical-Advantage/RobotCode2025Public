// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "Constants.h"
#include "thirdparty/junction/networktables/LoggedNetworkNumber.h"

namespace util {

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
class LoggedTunableNumber : public std::function<double()> {
public:
  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  explicit LoggedTunableNumber(const std::string &dashboardKey);

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  LoggedTunableNumber(const std::string &dashboardKey, double defaultValue);

  /**
   * Set the default value of the number. The default value can only be set
   * once.
   *
   * @param defaultValue The default value
   */
  void InitDefault(double defaultValue);

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  double Get() const;

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared
   * between multiple objects. Recommended approach is to pass the result of
   * "hashCode()"
   * @return True if the number has changed since the last time this method was
   * called, false otherwise.
   */
  bool HasChanged(int id);

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared
   * between multiple * objects. Recommended approach is to pass the result of
   * "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed.
   * Access tunable numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  static void
  IfChanged(int id, std::function<void(const std::vector<double> &)> action,
            const std::vector<LoggedTunableNumber *> &tunableNumbers);

  /** Runs action if any of the tunableNumbers have changed */
  static void
  IfChanged(int id, std::function<void()> action,
            const std::vector<LoggedTunableNumber *> &tunableNumbers);

  /** Overrides function call operator */
  double operator()() const override;

private:
  static constexpr char tableKey[] = "/Tuning";

  std::string key;
  bool hasDefault = false;
  double defaultValue = 0.0;
  std::unique_ptr<wpi::log::DataLogEntry> dashboardNumber;
  std::map<int, double> lastHasChangedValues;
};

} // namespace util