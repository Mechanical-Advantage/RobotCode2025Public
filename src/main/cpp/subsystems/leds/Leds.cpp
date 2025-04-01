// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/leds/Leds.h"

#include <cmath>
#include <numeric>

#include "frc/AddressableLED.h"
#include "frc/AddressableLEDBuffer.h"
#include "frc/DriverStation.h"
#include "frc/Notifier.h"
#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/util/Color.h"

#include "org/littletonrobotics/frc2025/FieldConstants.h"
#include "org/littletonrobotics/frc2025/util/LoggedTracer.h"

Leds Leds::instance;

Leds &Leds::GetInstance() {
  if (!instance) {
    instance = Leds();
  }
  return instance;
}

Leds::Leds()
    : leds(9), buffer(32), loadingNotifier([this] {
        std::lock_guard<std::mutex> lock(notifierMutex);
        breath(fullSection, frc::Color::kWhite, frc::Color::kBlack,
               breathSlowDuration,
               std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                       .count() /
                   1000.0);
        leds.SetData(buffer);
      }) {
  leds.SetLength(length);
  leds.SetData(buffer);
  leds.Start();
  loadingNotifier.StartPeriodic(0.02);
}

void Leds::periodic() {
  // Update alliance color
  if (frc::DriverStation::IsFMSAttached()) {
    alliance = frc::DriverStation::GetAlliance();
    disabledColor =
        alliance.has_value() &&
                alliance.value() == frc::DriverStation::Alliance::kBlue
            ? frc::Color::kBlue
            : frc::Color::kRed;
    secondaryDisabledColor =
        alliance.has_value() ? frc::Color::kBlack : secondaryDisabledColor;
  }

  // Update auto state
  if (frc::DriverStation::IsEnabled()) {
    lastEnabledAuto = frc::DriverStation::IsAutonomous();
    lastEnabledTime = frc::Timer::GetFPGATimestamp();
  }

  // Update estop state
  if (frc::DriverStation::IsEStopped()) {
    estopped = true;
  }

  // Exit during initial cycles
  loopCycleCount += 1;
  if (loopCycleCount < minLoopCycleCount) {
    return;
  }

  // Stop loading notifier if running
  loadingNotifier.Stop();

  // Strategy priorities
  hexColor =
      renderPriority(firstPriorityLevel, firstPriorityBlocked, topSection);
  secondaryHexColor =
      renderPriority(secondPriorityLevel, secondPriorityBlocked, bottomSection);

  // Select LED mode
  solid(fullSection, frc::Color::kBlack); // Default to off
  if (estopped) {
    solid(fullSection, frc::Color::kRed);
  } else if (frc::DriverStation::IsDisabled()) {
    if (superstructureCoast) {
      // Elevator coast alert
      solid(fullSection, frc::Color::kWhite);
    } else if (lastEnabledAuto &&
               frc::Timer::GetFPGATimestamp() - lastEnabledTime <
                   autoFadeMaxTime) {
      // Auto fade
      wave(Section{0, static_cast<int>(length *
                                       (1 - ((frc::Timer::GetFPGATimestamp() -
                                              lastEnabledTime) /
                                             autoFadeTime)))},
           frc::Color::kGold, frc::Color::kDarkBlue, waveFastCycleLength,
           waveFastDuration);
    } else if (lowBatteryAlert) {
      // Low battery
      strobe(fullSection, frc::Color::kOrangeRed, frc::Color::kBlack,
             strobeDuration);
    } else if (prideLeds) {
      // Pride stripes
      stripes(fullSection,
              {frc::Color::kBlack, frc::Color::kRed, frc::Color::kOrangeRed,
               frc::Color::kYellow, frc::Color::kGreen, frc::Color::kBlue,
               frc::Color::kPurple, frc::Color::kBlack,
               frc::Color(0.15, 0.3, 1.0), frc::Color::kDeepPink,
               frc::Color::kWhite, frc::Color::kDeepPink,
               frc::Color(0.15, 0.3, 1.0)},
              3, 5.0);
    } else {
      // Default pattern
      wave(fullSection, disabledColor, secondaryDisabledColor,
           waveDisabledCycleLength, waveDisabledDuration);
    }

    // Vision disconnected alert
    if (visionDisconnected) {
      strobe(bottomQuartSection, frc::Color::kRed, frc::Color::kBlack,
             strobeDuration);
    }

  } else if (frc::DriverStation::IsAutonomous()) {
    if (characterizationMode) {
      strobe(fullSection, frc::Color::kGold, frc::Color::kBlack, 0.5);
    } else {
      wave(fullSection, frc::Color::kGold, frc::Color::kDarkBlue,
           waveFastCycleLength, waveFastDuration);
    }
  } else {
    solid(topSection, hexColor);
    solid(bottomSection, secondaryHexColor);

    // Auto scoring reef
    if (autoScoringReef) {
      rainbow(topThreeQuartSection, rainbowCycleLength, rainbowDuration);
      solid(bottomQuartSection,
            autoScoringLevel == ReefLevel::L1   ? l1PriorityColor
            : autoScoringLevel == ReefLevel::L2 ? l2PriorityColor
            : autoScoringLevel == ReefLevel::L3 ? l3PriorityColor
            : autoScoringLevel == ReefLevel::L4 ? l4PriorityColor
                                                : frc::Color::kBlack);
    }

    // Auto scoring
    if (autoScoring) {
      rainbow(fullSection, rainbowCycleLength, rainbowDuration);
    }

    // Ready alert
    if (ready) {
      strobe(fullSection, frc::Color::kWhite, frc::Color::kBlue,
             strobeDuration);
    }

    // Coral grab alert
    if (coralGrabbed) {
      solid(fullSection, frc::Color::kLime);
    }

    // Human player alert
    if (hpAttentionAlert) {
      strobe(fullSection, frc::Color::kWhite, frc::Color::kBlack,
             strobeDuration);
    }

    // Endgame alert
    if (endgameAlert) {
      strobe(fullSection, frc::Color::kRed, frc::Color::kGold, strobeDuration);
    }
  }

  // Superstructure estop alert
  if (superstructureEstopped) {
    solid(fullSection, frc::Color::kRed);
  }

  // Update dashboard
  frc::SmartDashboard::PutString("LEDs/First Priority", hexColor.ToHexString());
  frc::SmartDashboard::PutString("LEDs/Second Priority",
                                 secondaryHexColor.ToHexString());

  // Update LEDs
  leds.SetData(buffer);

  // Record cycle time
  LoggedTracer::record("LEDs");
}

frc::Color Leds::solid(Section section, frc::Color color) {
  if (color.IsValid()) {
    for (int i = section.start; i < section.end; i++) {
      buffer.SetLED(i, color);
    }
  }
  return color;
}

frc::Color Leds::strobe(Section section, frc::Color c1, frc::Color c2,
                        double duration) {
  bool c1On =
      (std::fmod(frc::Timer::GetFPGATimestamp(), duration) / duration) > 0.5;
  return solid(section, c1On ? c1 : c2);
}

frc::Color Leds::breath(Section section, frc::Color c1, frc::Color c2,
                        double duration, double timestamp) {
  double x = std::fmod(timestamp, duration) / duration * 2.0 * M_PI;
  double ratio = (std::sin(x) + 1.0) / 2.0;
  double red = (c1.red() * (1 - ratio)) + (c2.red() * ratio);
  double green = (c1.green() * (1 - ratio)) + (c2.green() * ratio);
  double blue = (c1.blue() * (1 - ratio)) + (c2.blue() * ratio);
  frc::Color color(red, green, blue);
  solid(section, color);
  return color;
}

frc::Color Leds::breathCalculate(Section section, frc::Color c1, frc::Color c2,
                                 double duration) {
  double x = std::fmod(frc::Timer::GetFPGATimestamp(), duration) / duration *
             2.0 * M_PI;
  double ratio = (std::sin(x) + 1.0) / 2.0;
  double red = (c1.red() * (1 - ratio)) + (c2.red() * ratio);
  double green = (c1.green() * (1 - ratio)) + (c2.green() * ratio);
  double blue = (c1.blue() * (1 - ratio)) + (c2.blue() * ratio);
  frc::Color color(red, green, blue);
  return color;
}

frc::Color Leds::breath(Section section, frc::Color c1, frc::Color c2,
                        double duration) {
  return breath(section, c1, c2, duration, frc::Timer::GetFPGATimestamp());
}

void Leds::rainbow(Section section, double cycleLength, double duration) {
  double x =
      (1 - std::fmod(frc::Timer::GetFPGATimestamp() / duration, 1.0)) * 180.0;
  double xDiffPerLed = 180.0 / cycleLength;
  for (int i = section.end - 1; i >= section.start; i--) {
    x += xDiffPerLed;
    x = std::fmod(x, 180.0);
    buffer.SetHSV(i, static_cast<int>(x), 255, 255);
  }
}

void Leds::wave(Section section, frc::Color c1, frc::Color c2,
                double cycleLength, double duration) {
  double x =
      (1 - std::fmod(frc::Timer::GetFPGATimestamp(), duration) / duration) *
      2.0 * M_PI;
  double xDiffPerLed = (2.0 * M_PI) / cycleLength;
  for (int i = section.end - 1; i >= section.start; i--) {
    x += xDiffPerLed;
    double ratio = (std::pow(std::sin(x), waveExponent) + 1.0) / 2.0;
    if (std::isnan(ratio)) {
      ratio = (-std::pow(std::sin(x + M_PI), waveExponent) + 1.0) / 2.0;
    }
    if (std::isnan(ratio)) {
      ratio = 0.5;
    }
    double red = (c1.red() * (1 - ratio)) + (c2.red() * ratio);
    double green = (c1.green() * (1 - ratio)) + (c2.green() * ratio);
    double blue = (c1.blue() * (1 - ratio)) + (c2.blue() * ratio);
    buffer.SetLED(i, frc::Color(red, green, blue));
  }
}

void Leds::stripes(Section section, const std::list<frc::Color> &colors,
                   int stripeLength, double duration) {
  int offset =
      static_cast<int>(std::fmod(frc::Timer::GetFPGATimestamp(), duration) /
                       duration * stripeLength * colors.size());
  for (int i = section.end - 1; i >= section.start; i--) {
    int colorIndex =
        static_cast<int>(
            std::floor(static_cast<double>(i - offset) / stripeLength) +
            colors.size()) %
        colors.size();
    colorIndex = colors.size() - 1 - colorIndex;
    auto it = colors.begin();
    std::advance(it, colorIndex);
    buffer.SetLED(i, *it);
  }
}

frc::Color Leds::renderPriority(std::optional<ReefLevel> level, bool blocked,
                                Section section) {
  frc::Color primaryColor =
      level.has_value()
          ? (level.value() == ReefLevel::L1   ? l1PriorityColor
             : level.value() == ReefLevel::L2 ? l2PriorityColor
             : level.value() == ReefLevel::L3 ? l3PriorityColor
             : level.value() == ReefLevel::L4 ? l4PriorityColor
                                              : frc::Color::kBlack)
          : frc::Color::kBlack;

  if (!blocked) {
    return primaryColor;
  } else {
    return breathCalculate(
        section, primaryColor,
        frc::Color::LerpRGB(primaryColor, frc::Color::kBlack, 0.9),
        breathFastDuration);
  }
}
