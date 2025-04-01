// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <list>
#include <optional>
#include <vector>

#include "frc/AddressableLED.h"
#include "frc/AddressableLEDBuffer.h"
#include "frc/DriverStation.h"
#include "frc/Notifier.h"
#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/util/Color.h"

#include "org/littletonrobotics/frc2025/FieldConstants.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"
#include "org/littletonrobotics/frc2025/util/VirtualSubsystem.h"

class Leds : public VirtualSubsystem {
public:
  static Leds &GetInstance();

  void periodic() override;

  int loopCycleCount = 0;
  bool hpAttentionAlert = false;
  bool endgameAlert = false;
  bool autoScoringReef = false;
  bool autoScoring = false;
  bool superstructureCoast = false;
  bool superstructureEstopped = false;
  bool lowBatteryAlert = false;
  bool characterizationMode = false;
  bool visionDisconnected = false;
  bool ready = false;
  bool coralGrabbed = false;
  std::optional<ReefLevel> firstPriorityLevel = std::nullopt;
  std::optional<ReefLevel> secondPriorityLevel = std::nullopt;
  ReefLevel autoScoringLevel = ReefLevel::L4;
  bool firstPriorityBlocked = false;
  bool secondPriorityBlocked = false;
  frc::Color hexColor = frc::Color::kBlack;
  frc::Color secondaryHexColor = frc::Color::kBlack;

private:
  Leds();
  ~Leds() = default;

  std::optional<frc::DriverStation::Alliance> alliance = std::nullopt;
  frc::Color disabledColor = frc::Color::kGold;
  frc::Color secondaryDisabledColor = frc::Color::kDarkBlue;
  bool lastEnabledAuto = false;
  double lastEnabledTime = 0.0;
  bool estopped = false;

  frc::AddressableLED leds{9};
  frc::AddressableLEDBuffer buffer{32};
  frc::Notifier loadingNotifier;

  static constexpr bool prideLeds = false;
  static constexpr int minLoopCycleCount = 10;
  static constexpr int length = 32;
  struct Section {
    int start;
    int end;
  };
  static constexpr Section fullSection = {0, length};
  static constexpr Section topSection = {length / 2, length};
  static constexpr Section bottomSection = {0, length / 2};
  static constexpr Section topThreeQuartSection = {length / 4, length};
  static constexpr Section bottomQuartSection = {0, length / 4};
  static constexpr double strobeDuration = 0.1;
  static constexpr double breathFastDuration = 0.5;
  static constexpr double breathSlowDuration = 1.0;
  static constexpr double rainbowCycleLength = 25.0;
  static constexpr double rainbowDuration = 0.25;
  static constexpr double waveExponent = 0.4;
  static constexpr double waveFastCycleLength = 25.0;
  static constexpr double waveFastDuration = 0.25;
  static constexpr double waveDisabledCycleLength = 15.0;
  static constexpr double waveDisabledDuration = 2.0;
  static constexpr double autoFadeTime = 2.5;
  static constexpr double autoFadeMaxTime = 5.0;
  static constexpr frc::Color l1PriorityColor = frc::Color::kOrangeRed;
  static constexpr frc::Color l2PriorityColor = frc::Color::kCyan;
  static constexpr frc::Color l3PriorityColor = frc::Color::kBlue;
  static constexpr frc::Color l4PriorityColor = frc::Color::kPurple;

  frc::Color solid(Section section, frc::Color color);
  frc::Color strobe(Section section, frc::Color c1, frc::Color c2,
                    double duration);
  frc::Color breath(Section section, frc::Color c1, frc::Color c2,
                    double duration, double timestamp);
  frc::Color breathCalculate(Section section, frc::Color c1, frc::Color c2,
                             double duration);
  frc::Color breath(Section section, frc::Color c1, frc::Color c2,
                    double duration);
  void rainbow(Section section, double cycleLength, double duration);
  void wave(Section section, frc::Color c1, frc::Color c2, double cycleLength,
            double duration);
  void stripes(Section section, const std::list<frc::Color> &colors,
               int stripeLength, double duration);
  frc::Color renderPriority(std::optional<ReefLevel> level, bool blocked,
                            Section section);

  static Leds instance;
};