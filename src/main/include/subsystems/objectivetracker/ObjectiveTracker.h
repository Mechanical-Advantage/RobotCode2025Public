// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <functional>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "frc/DriverStation.h"
#include "frc/math/geometry/Pose2d.h"
#include "frc/math/geometry/Pose3d.h"
#include "frc/math/geometry/Rotation3d.h"
#include "frc/math/geometry/Translation3d.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringPublisher.h"
#include "org/littletonrobotics/frc2025/FieldConstants.h"
#include "org/littletonrobotics/frc2025/subsystems/objectivetracker/ReefControlsIO.h"
#include "org/littletonrobotics/frc2025/util/LoggedTunableNumber.h"
#include "org/littletonrobotics/frc2025/util/VirtualSubsystem.h"
#include "org/littletonrobotics/junction/AutoLog.h"
#include "org/littletonrobotics/junction/networktables/LoggedDashboardChooser.h"
#include "org/littletonrobotics/junction/networktables/LoggedNetworkString.h"

class ObjectiveTracker : public VirtualSubsystem {
public:
  ObjectiveTracker(ReefControlsIO &io);
  ~ObjectiveTracker() override = default;

  void Periodic() override;

  std::optional<FieldConstants::ReefLevel> GetFirstLevel() const {
    return firstLevel;
  }
  std::optional<FieldConstants::ReefLevel> GetSecondLevel() const {
    return secondLevel;
  }
  std::optional<FieldConstants::AlgaeObjective> GetAlgaeObjective() const {
    return algaeObjective;
  }

  void SetForceReefBlocked(std::function<bool()> forceReefBlocked);

  std::optional<FieldConstants::CoralObjective>
  GetCoralObjective(FieldConstants::ReefLevel level);

private:
  static constexpr char inactiveColor[] = "#242424";
  static constexpr char completeColor[] = "#00ff00";
  static constexpr char incompleteColor[] = "#ff0000";
  static constexpr char skipColor[] = "#ffff00";

  static const std::set<FieldConstants::CoralObjective> l1CoralObjectives;

  static LoggedTunableNumber lookaheadS;

  ReefControlsIO &io;
  ReefControlsIO::ReefControlsIOInputs inputs;

  struct ReefState {
    std::array<std::array<bool, 12>, 3> coral;
    std::array<bool, 6> algae;
    int troughCount;

    static const ReefState initial;

    bool operator==(const ReefState &other) const {
      return troughCount == other.troughCount && algae == other.algae &&
             coral == other.coral;
    }

    ReefState Clone() const { return ReefState{coral, algae, troughCount}; }
  };

  ReefState reefState = ReefState::initial;
  ReefState previousReefState = ReefState::initial;
  bool coopState = false;
  int selectedLevel = 0;
  bool lastCoopState = false;

  std::set<FieldConstants::CoralObjective> availableBranches;
  std::set<FieldConstants::CoralObjective> availableUnblockedBranches;
  std::set<FieldConstants::CoralObjective> nearbyBranches;
  std::set<FieldConstants::CoralObjective> nearbyUnblockedBranches;

  std::set<FieldConstants::AlgaeObjective> presentAlgae;

  std::vector<CoralPriority> fullStrategy;
  std::vector<CoralPriority> uncompletedPriorities;
  std::vector<CoralPriority> openPriorities;

  std::optional<FieldConstants::ReefLevel> firstLevel;
  std::optional<FieldConstants::ReefLevel> secondLevel;
  std::optional<FieldConstants::AlgaeObjective> algaeObjective;

  LoggedNetworkString strategyInput =
      LoggedNetworkString("/SmartDashboard/Strategy", "545352514321");
  std::string previousStrategy;
  std::array<nt::StringPublisher, 8> strategyNamesPublishers;
  std::array<nt::StringPublisher, 8> strategyCompletedPublishers;
  LoggedDashboardChooser<std::string> dashboardLevelChooser =
      LoggedDashboardChooser<std::string>("Reef Level");

  frc::Pose2d predictedRobot;

  std::function<bool()> forceReefBlocked;

  void ParseStrategy();
  std::optional<FieldConstants::ReefLevel> GetLevel(bool secondPriority);
  static void LogAvailableBranches(
      const std::set<FieldConstants::CoralObjective> &availableBranches,
      const std::string &key);
  static std::function<bool(const FieldConstants::CoralObjective &,
                            const FieldConstants::CoralObjective &)>
  NearestCoralObjectiveComparator(const frc::Pose2d &robot);

  AUTO_LOG_OUTPUT(predictedRobot);

  struct CoralPriority {
    FieldConstants::ReefLevel level;
    bool fill;
    std::string name;

    CoralPriority(FieldConstants::ReefLevel level, bool fill, std::string name)
        : level(level), fill(fill), name(name) {}

    bool Complete(const ReefState &reefState) const {
      if (fill) {
        if (level == FieldConstants::ReefLevel::L1)
          return false;
        const std::array<bool, 12> &levelState =
            reefState.coral[level.ordinal() - 1];
        for (bool b : levelState) {
          if (!b)
            return false;
        }
        return true;
      }
      if (level == FieldConstants::ReefLevel::L1)
        return reefState.troughCount >= 5;
      int count = 0;
      const std::array<bool, 12> &levelState =
          reefState.coral[level.ordinal() - 1];
      for (bool b : levelState) {
        if (b)
          count++;
      }
      return count >= 5;
    }

    static const CoralPriority _54;
    static const CoralPriority _53;
    static const CoralPriority _52;
    static const CoralPriority _51;
    static const CoralPriority _4;
    static const CoralPriority _3;
    static const CoralPriority _2;
    static const CoralPriority _1;
  };
};