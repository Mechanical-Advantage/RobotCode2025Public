// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/objectivetracker/ObjectiveTracker.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

#include "frc/DriverStation.h"
#include "frc/math/geometry/Pose2d.h"
#include "frc/math/geometry/Pose3d.h"
#include "frc/math/geometry/Rotation3d.h"
#include "frc/math/geometry/Translation3d.h"
#include "frc/math/util/Units.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringPublisher.h"
#include "org/littletonrobotics/frc2025/FieldConstants.h"
#include "org/littletonrobotics/frc2025/RobotState.h"
#include "org/littletonrobotics/frc2025/commands/AutoScoreCommands.h"
#include "org/littletonrobotics/frc2025/subsystems/leds/Leds.h"
#include "org/littletonrobotics/frc2025/util/AllianceFlipUtil.h"
#include "org/littletonrobotics/frc2025/util/GeomUtil.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/junction/Logger.h"

using namespace ObjectiveTracker;

const std::set<FieldConstants::CoralObjective>
    ObjectiveTracker::l1CoralObjectives = []() {
      std::set<FieldConstants::CoralObjective> result;
      for (int id = 0; id <= 11; ++id) {
        result.emplace(id, FieldConstants::ReefLevel::L1);
      }
      return result;
    }();

LoggedTunableNumber ObjectiveTracker::lookaheadS =
    LoggedTunableNumber("ObjectiveTracker/LookaheadS", 0.15);

ObjectiveTracker::ObjectiveTracker(ReefControlsIO &io) : io(io) {
  dashboardLevelChooser.AddDefaultOption("   Auto   ", "Auto");
  for (const auto &level : FieldConstants::ReefLevelValues) {
    dashboardLevelChooser.AddOption(
        [&level]() {
          switch (level) {
          case FieldConstants::ReefLevel::L1:
            return "   Level 1   ";
          case FieldConstants::ReefLevel::L2:
            return "   Level 2   ";
          case FieldConstants::ReefLevel::L3:
            return "   Level 3   ";
          case FieldConstants::ReefLevel::L4:
            return "   Level 4   ";
          default:
            return "";
          }
        }(),
        FieldConstants::ReefLevelToString(level));
  }

  auto table = nt::NetworkTableInstance::GetDefault()
                   .GetTable("SmartDashboard")
                   .GetSubTable("Objective Tracker");
  for (int i = 0; i < 8; ++i) {
    strategyNamesPublishers[i] =
        table->GetStringTopic("Priority #" + std::to_string(i + 1)).Publish();
    strategyCompletedPublishers[i] =
        table
            ->GetStringTopic("Priority #" + std::to_string(i + 1) +
                             " Completed")
            .Publish();

    strategyNamesPublishers[i].Set("");
    strategyCompletedPublishers[i].Set(inactiveColor);
  }
}

void ObjectiveTracker::Periodic() {
  io.UpdateInputs(inputs);
  Logger::ProcessInputs("ReefControls", inputs);

  bool strategyChanged = false;
  if (strategyInput.Get() != previousStrategy) {
    strategyChanged = true;
    ParseStrategy();
  }

  if (!inputs.level1State.empty()) {
    reefState =
        ReefState{reefState.coral, reefState.algae, inputs.level1State[0]};
  }
  std::array<std::vector<int>, 3> inputLevelStates = {
      inputs.level2State, inputs.level3State, inputs.level4State};
  for (int i = 0; i < 3; ++i) {
    if (!inputLevelStates[i].empty()) {
      std::array<bool, 12> levelState;
      for (int j = 0; j < 12; ++j) {
        levelState[j] = (inputLevelStates[i][0] & (1 << j)) != 0;
      }
      reefState.coral[i] = levelState;
    }
  }
  if (!inputs.algaeState.empty()) {
    std::array<bool, 6> algae;
    for (int j = 0; j < 6; ++j) {
      algae[j] = (inputs.algaeState[0] & (1 << j)) != 0;
    }
    reefState = ReefState{reefState.coral, algae, reefState.troughCount};
  }
  if (!inputs.coopState.empty()) {
    coopState = inputs.coopState[0];
  }
  if (frc::DriverStation::GetMatchType() ==
      frc::DriverStation::MatchType::kElimination) {
    coopState = false;
  }

  if (!(reefState == previousReefState) || coopState != lastCoopState ||
      strategyChanged) {
    previousReefState = reefState.Clone();
    lastCoopState = coopState;

    availableBranches.clear();
    availableUnblockedBranches.clear();
    availableBranches.insert(l1CoralObjectives.begin(),
                             l1CoralObjectives.end());
    availableUnblockedBranches.insert(l1CoralObjectives.begin(),
                                      l1CoralObjectives.end());
    for (int level = 0; level < 3; ++level) {
      for (int branch = 0; branch < 12; ++branch) {
        if (reefState.coral[level][branch])
          continue;
        FieldConstants::ReefLevel reefLevel =
            FieldConstants::ReefLevelFromString("L" +
                                                std::to_string(level + 2));
        availableBranches.emplace(branch, reefLevel);

        if (level != 2) {
          int face = branch / 2;
          if (level == 1 && reefState.algae[face])
            continue;
          else if (face % 2 == 1 && reefState.algae[face])
            continue;
        }
        availableUnblockedBranches.emplace(branch, reefLevel);
      }
    }
    LogAvailableBranches(availableBranches, "AvailableBranches");
    LogAvailableBranches(availableUnblockedBranches,
                         "AvailableUnblockedBranches");

    presentAlgae.clear();
    for (int i = 0; i < 6; ++i) {
      if (reefState.algae[i])
        presentAlgae.emplace(i);
    }
    if (presentAlgae.empty()) {
      Logger::RecordOutput("ObjectiveTracker/ReefState/PresentAlgae",
                           std::vector<FieldConstants::AlgaeObjective>{});
    } else {
      std::vector<FieldConstants::AlgaeObjective> algaeVec(presentAlgae.begin(),
                                                           presentAlgae.end());
      Logger::RecordOutput("ObjectiveTracker/ReefState/PresentAlgae", algaeVec);
    }

    uncompletedPriorities.clear();
    std::optional<CoralPriority> uselessPriority;
    if (coopState) {
      std::set<CoralPriority> coopPriorities = {
          CoralPriority::_54, CoralPriority::_53, CoralPriority::_52,
          CoralPriority::_51};
      auto it = std::min_element(
          coopPriorities.begin(), coopPriorities.end(),
          [&](const CoralPriority &a, const CoralPriority &b) {
            if (a.Complete(reefState))
              return false;
            if (b.Complete(reefState))
              return true;

            auto aIt = std::find(fullStrategy.begin(), fullStrategy.end(), a);
            auto bIt = std::find(fullStrategy.begin(), fullStrategy.end(), b);
            int aIndex = (aIt == fullStrategy.end())
                             ? INT_MAX
                             : std::distance(fullStrategy.begin(), aIt);
            int bIndex = (bIt == fullStrategy.end())
                             ? INT_MAX
                             : std::distance(fullStrategy.begin(), bIt);
            if (aIndex != bIndex)
              return bIndex < aIndex;

            if (a.level == FieldConstants::ReefLevel::L1 &&
                b.level != FieldConstants::ReefLevel::L1)
              return true;
            if (b.level == FieldConstants::ReefLevel::L1 &&
                a.level != FieldConstants::ReefLevel::L1)
              return false;

            int aCount = std::accumulate(
                reefState.coral[a.level.ordinal() - 1].begin(),
                reefState.coral[a.level.ordinal() - 1].end(), 0);
            int bCount = std::accumulate(
                reefState.coral[b.level.ordinal() - 1].begin(),
                reefState.coral[b.level.ordinal() - 1].end(), 0);
            return aCount < bCount;
          });
      if (!(*it).Complete(reefState)) {
        uselessPriority = *it;
      }
    }

    for (int i = 0; i < fullStrategy.size(); ++i) {
      if (uselessPriority.has_value() &&
          uselessPriority.value() == fullStrategy[i]) {
        strategyCompletedPublishers[i].Set(skipColor);
      } else if (fullStrategy[i].Complete(reefState)) {
        strategyCompletedPublishers[i].Set(completeColor);
      } else {
        strategyCompletedPublishers[i].Set(incompleteColor);
        uncompletedPriorities.push_back(fullStrategy[i]);
      }
    }

    for (int i = 0; i < reefState.coral.size(); ++i) {
      for (int j = 0; j < reefState.coral[i].size(); ++j) {
        Logger::RecordOutput("ObjectiveTracker/ReefState/CoralState/Level" +
                                 std::to_string(i + 2) + "/Branch" +
                                 std::to_string(j + 1),
                             reefState.coral[i][j]);
      }
    }
    for (int i = 0; i < reefState.algae.size(); ++i) {
      Logger::RecordOutput("ObjectiveTracker/ReefState/AlgaeState/" +
                               std::to_string(i + 1),
                           reefState.algae[i]);
    }

    std::set<frc::Pose3d> coralPoses;
    for (int level = 0; level < reefState.coral.size(); ++level) {
      for (int j = 0; j < reefState.coral[0].size(); ++j) {
        if (!reefState.coral[level][j])
          continue;
        coralPoses.insert(AllianceFlipUtil::Apply(
            FieldConstants::Reef::branchPositions[j][FieldConstants::ReefLevel(
                                                         level + 1)]
                .TransformBy(GeomUtil::ToTransform3d(frc::Pose3d(
                    frc::Translation3d(
                        -frc::Units::InchesToMeters(11.875) / 2.5, 0.0, 0.0),
                    frc::Rotation3d::Identity())))));
      }
    }
    std::vector<frc::Pose3d> coralPosesVec(coralPoses.begin(),
                                           coralPoses.end());
    Logger::RecordOutput("ObjectiveTracker/3DView/Coral", coralPosesVec);

    std::set<frc::Translation3d> algaePoses;
    for (int i = 0; i < 6; ++i) {
      if (!reefState.algae[i])
        continue;
      auto firstBranchPose =
          FieldConstants::Reef::branchPositions[i * 2]
                                               [FieldConstants::ReefLevel::L2];
      auto secondBranchPose =
          FieldConstants::Reef::branchPositions[i * 2 + 1]
                                               [FieldConstants::ReefLevel::L3];
      algaePoses.insert(AllianceFlipUtil::Apply(
          firstBranchPose.Translation()
              .Interpolate(secondBranchPose.Translation(), 0.5)
              .Plus(frc::Translation3d(
                  -FieldConstants::algaeDiameter / 3.0,
                  frc::Rotation3d(0.0, -35.0 / 180.0 * M_PI,
                                  firstBranchPose.Rotation().Z()),
                  0.0))
              .Plus(frc::Translation3d(0.0, 0.0,
                                       (i % 2 == 0) ? secondBranchPose.Z() -
                                                          firstBranchPose.Z()
                                                    : 0.0))));
    }
    std::vector<frc::Translation3d> algaePosesVec(algaePoses.begin(),
                                                  algaePoses.end());
    Logger::RecordOutput("ObjectiveTracker/3DView/Algae", algaePosesVec);
  }

  for (int i = 0; i < fullStrategy.size(); ++i) {
    strategyNamesPublishers[i].Set(fullStrategy[i].name);
  }
  for (int i = fullStrategy.size(); i < 8; ++i) {
    strategyNamesPublishers[i].Set("");
  }

  if (!inputs.selectedLevel.empty()) {
    selectedLevel = inputs.selectedLevel[0];
  }
  io.SetSelectedLevel(selectedLevel);
  io.SetLevel1State(reefState.troughCount);
  std::array<int, 3> levelStates = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 12; ++j) {
      if (reefState.coral[i][j]) {
        levelStates[i] |= 1 << j;
      }
    }
  }
  io.SetLevel2State(levelStates[0]);
  io.SetLevel3State(levelStates[1]);
  io.SetLevel4State(levelStates[2]);
  int algaeState = 0;
  for (int i = 0; i < 6; ++i) {
    if (reefState.algae[i]) {
      algaeState |= 1 << i;
    }
  }
  io.SetAlgaeState(algaeState);
  io.SetCoopState(coopState);
  io.SetElims(frc::DriverStation::GetMatchType() ==
              frc::DriverStation::MatchType::kElimination);

  predictedRobot =
      AllianceFlipUtil::Apply(RobotState::GetInstance().GetEstimatedPose().Exp(
          RobotState::GetInstance().GetRobotVelocity().ToTwist2d(
              lookaheadS.Get())));

  nearbyBranches.clear();
  for (const auto &objective : availableBranches) {
    if (std::abs(predictedRobot
                     .RelativeTo(AutoScoreCommands::GetBranchPose(objective))
                     .Translation()
                     .Angle()
                     .Degrees()) <= 100.0) {
      nearbyBranches.insert(objective);
    }
  }

  openPriorities.clear();
  int count = 0;
  for (const auto &priority : uncompletedPriorities) {
    bool foundLevel = false;
    for (const auto &openPriority : openPriorities) {
      if (priority.level == openPriority.level) {
        foundLevel = true;
        break;
      }
    }

    bool foundNearby = false;
    for (const auto &nearbyBranch : nearbyBranches) {
      if (priority.level == nearbyBranch.reefLevel()) {
        foundNearby = true;
        break;
      }
    }

    if (!foundLevel && foundNearby) {
      openPriorities.push_back(priority);
      count++;
    }
    if (count == 2)
      break;
  }
  std::vector<CoralPriority> openPrioritiesVec(openPriorities.begin(),
                                               openPriorities.end());
  Logger::RecordOutput("ObjectiveTracker/Strategy/OpenPriorities",
                       openPrioritiesVec);

  firstLevel = GetLevel(false);
  secondLevel = GetLevel(true);

  nearbyUnblockedBranches.clear();
  if (!forceReefBlocked()) {
    for (const auto &objective : nearbyBranches) {
      if (availableUnblockedBranches.count(objective)) {
        nearbyUnblockedBranches.insert(objective);
      }
    }
  }

  Leds::GetInstance().firstPriorityLevel = firstLevel;
  Leds::GetInstance().secondPriorityLevel = secondLevel;
  Leds::GetInstance().firstPriorityBlocked =
      firstLevel.has_value()
          ? std::none_of(nearbyUnblockedBranches.begin(),
                         nearbyUnblockedBranches.end(),
                         [&](const FieldConstants::CoralObjective &objective) {
                           return firstLevel.value() == objective.reefLevel();
                         })
          : true;
  Leds::GetInstance().secondPriorityBlocked =
      secondLevel.has_value()
          ? std::none_of(nearbyUnblockedBranches.begin(),
                         nearbyUnblockedBranches.end(),
                         [&](const FieldConstants::CoralObjective &objective) {
                           return secondLevel.value() == objective.reefLevel();
                         })
          : true;

  algaeObjective = std::nullopt;
  for (const auto &objective : presentAlgae) {
    if (predictedRobot
            .RelativeTo(AutoScoreCommands::GetReefIntakePose(objective))
            .X() <= 0.1) {
      if (!algaeObjective.has_value() ||
          predictedRobot.Translation().Distance(
              AutoScoreCommands::GetReefIntakePose(objective).Translation()) <
              predictedRobot.Translation().Distance(
                  AutoScoreCommands::GetReefIntakePose(algaeObjective.value())
                      .Translation())) {
        algaeObjective = objective;
      }
    }
  }
  if (algaeObjective.has_value()) {
    Logger::RecordOutput(
        "ObjectiveTracker/Strategy/AlgaeObjective",
        std::vector<FieldConstants::AlgaeObjective>{algaeObjective.value()});
  } else {
    Logger::RecordOutput("ObjectiveTracker/Strategy/AlgaeObjective",
                         std::vector<FieldConstants::AlgaeObjective>{});
  }

  LoggedTracer::Record("ObjectiveTracker");
}

std::optional<FieldConstants::ReefLevel>
ObjectiveTracker::GetLevel(bool secondPriority) {
  if (dashboardLevelChooser.Get() != "Auto") {
    return FieldConstants::ReefLevelFromString(dashboardLevelChooser.Get());
  }
  if (!secondPriority) {
    return !openPriorities.empty() ? openPriorities[0].level : std::nullopt;
  } else {
    return openPriorities.size() < 2 ? std::nullopt : openPriorities[1].level;
  }
}

std::optional<FieldConstants::CoralObjective>
ObjectiveTracker::GetCoralObjective(FieldConstants::ReefLevel level) {
  auto comparator = NearestCoralObjectiveComparator(predictedRobot);
  std::optional<FieldConstants::CoralObjective> result;
  for (const auto &objective : nearbyUnblockedBranches) {
    if (objective.reefLevel() == level) {
      if (!result.has_value() || comparator(objective, result.value())) {
        result = objective;
      }
    }
  }
  return result;
}

const std::set<char> allowedCharacters = {'1', '2', '3', '4', '5'};

void ObjectiveTracker::ParseStrategy() {
  previousStrategy = strategyInput.Get();
  fullStrategy.clear();
  std::stringstream filtered;
  for (char chr : strategyInput.Get()) {
    if (allowedCharacters.count(chr)) {
      filtered << chr;
    }
  }
  Logger::RecordOutput("ObjectiveTracker/Strategy/StrategyInput",
                       filtered.str());
  std::string filteredStr = filtered.str();
  for (int i = 0; i < filteredStr.length(); ++i) {
    if (filteredStr[i] == '5' && i + 1 < filteredStr.length() &&
        filteredStr[i + 1] != '5') {
      fullStrategy.push_back(
          CoralPriority::FromString("_" + filteredStr.substr(i, 2)));
      ++i;
    } else if (filteredStr[i] != '5') {
      fullStrategy.push_back(
          CoralPriority::FromString("_" + std::string(1, filteredStr[i])));
    }
  }
  std::set<CoralPriority> unique(fullStrategy.begin(), fullStrategy.end());
  fullStrategy.assign(unique.begin(), unique.end());
  for (int i = 4; i >= 1; --i) {
    CoralPriority priority = CoralPriority::FromString("_" + std::to_string(i));
    if (std::find(fullStrategy.begin(), fullStrategy.end(), priority) ==
        fullStrategy.end()) {
      fullStrategy.push_back(priority);
    }
  }
  Logger::RecordOutput("ObjectiveTracker/Strategy/FullStrategy", fullStrategy);

  for (int i = 0; i < fullStrategy.size(); ++i) {
    strategyNamesPublishers[i].Set(fullStrategy[i].name);
    strategyCompletedPublishers[i].Set(inactiveColor);
  }
  for (int i = fullStrategy.size(); i < 8; ++i) {
    strategyNamesPublishers[i].Set("");
    strategyCompletedPublishers[i].Set(inactiveColor);
  }
}

void ObjectiveTracker::LogAvailableBranches(
    const std::set<FieldConstants::CoralObjective> &availableBranches,
    const std::string &key) {
  std::map<FieldConstants::ReefLevel,
           std::vector<FieldConstants::CoralObjective>>
      branchesForLevel;
  for (const auto &objective : availableBranches) {
    branchesForLevel[objective.reefLevel()].push_back(objective);
  }
  for (const auto &pair : branchesForLevel) {
    Logger::RecordOutput("ObjectiveTracker/ReefState/" + key + "/Level" +
                             std::to_string(pair.first.ordinal() + 1),
                         pair.second);
  }
  for (const auto &level : FieldConstants::ReefLevelValues) {
    if (branchesForLevel.find(level) == branchesForLevel.end()) {
      Logger::RecordOutput("ObjectiveTracker/ReefState/" + key + "/Level" +
                               std::to_string(level.ordinal() + 1),
                           std::vector<FieldConstants::CoralObjective>{});
    }
  }
}

std::function<bool(const FieldConstants::CoralObjective &,
                   const FieldConstants::CoralObjective &)>
ObjectiveTracker::NearestCoralObjectiveComparator(const frc::Pose2d &robot) {
  return [&robot](const FieldConstants::CoralObjective &a,
                  const FieldConstants::CoralObjective &b) {
    return robot.Translation().Distance(
               AutoScoreCommands::GetCoralScorePose(a).Translation()) <
           robot.Translation().Distance(
               AutoScoreCommands::GetCoralScorePose(b).Translation());
  };
}

const ObjectiveTracker::ReefState ObjectiveTracker::ReefState::initial = {
    {{false, false, false, false, false, false, false, false, false, false,
      false, false},
     {false, false, false, false, false, false, false, false, false, false,
      false, false},
     {false, false, false, false, false, false, false, false, false, false,
      false, false}},
    {true, true, true, true, true, true},
    0};

ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_54 = {
    FieldConstants::ReefLevel::L4, false, "5/4"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_53 = {
    FieldConstants::ReefLevel::L3, false, "5/3"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_52 = {
    FieldConstants::ReefLevel::L2, false, "5/2"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_51 = {
    FieldConstants::ReefLevel::L1, false, "5/1"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_4 = {
    FieldConstants::ReefLevel::L4, true, "4"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_3 = {
    FieldConstants::ReefLevel::L3, true, "3"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_2 = {
    FieldConstants::ReefLevel::L2, true, "2"};
ObjectiveTracker::CoralPriority ObjectiveTracker::CoralPriority::_1 = {
    FieldConstants::ReefLevel::L1, true, "1"};

ObjectiveTracker::CoralPriority
ObjectiveTracker::CoralPriority::FromString(const std::string &str) {
  if (str == "_54")
    return _54;
  if (str == "_53")
    return _53;
  if (str == "_52")
    return _52;
  if (str == "_51")
    return _51;
  if (str == "_4")
    return _4;
  if (str == "_3")
    return _3;
  if (str == "_2")
    return _2;
  if (str == "_1")
    return _1;
  return _1;
}
