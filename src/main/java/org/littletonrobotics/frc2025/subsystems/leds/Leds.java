// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.VirtualSubsystem;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hpAttentionAlert = false;
  public boolean endgameAlert = false;
  public boolean autoScoringReef = false;
  public boolean autoScoring = false;
  public boolean superAutoScoring = false;
  public boolean superstructureCoast = false;
  public boolean superstructureEstopped = false;
  public boolean lowBatteryAlert = false;
  public boolean characterizationMode = false;
  public boolean visionDisconnected = false;
  public boolean ready = false;
  public boolean superClimbed = false;
  public boolean coralGrabbed = false;
  public Optional<ReefLevel> firstPriorityLevel = Optional.empty();
  public Optional<ReefLevel> secondPriorityLevel = Optional.empty();
  public ReefLevel autoScoringLevel = ReefLevel.L4;
  public boolean firstPriorityBlocked = false;
  public boolean secondPriorityBlocked = false;
  public Color firstPriorityColor = Color.kBlack;
  public Color secondPriorityColor = Color.kBlack;
  @Setter private BooleanSupplier shouldDimSupplier = () -> false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color disabledColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int fullLength = 42;
  private static final int length = (int) Math.ceil(fullLength / 2.0);
  private static final int sideSectionLength = 6;
  private static final int virtualSectionLength = 7;
  private static final int virtualLength = length + virtualSectionLength;
  private static final Section virtualSection =
      new Section(sideSectionLength, sideSectionLength + virtualSectionLength);
  private static final Section fullSection = new Section(0, virtualLength);
  private static final Section firstPrioritySection = new Section(0, 4 * virtualLength / 5);
  private static final Section secondPrioritySection =
      new Section(4 * virtualLength / 5, virtualLength);
  private static final Section straightSection = new Section(sideSectionLength, virtualLength);
  private static final Section sideSection = new Section(0, sideSectionLength);
  private static final double strobeDuration = 0.1;
  private static final double breathFastDuration = 0.5;
  private static final double breathSlowDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double rainbowStrobeDuration = 0.2;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveDisabledCycleLength = 15.0;
  private static final double waveDisabledDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal
  private static final Color l1PriorityColor = Color.kOrangeRed;
  private static final Color l2PriorityColor = Color.kCyan;
  private static final Color l3PriorityColor = Color.kBlue;
  private static final Color l4PriorityColor = Color.kPurple;
  private static final double dimMultiplier = 0.1;

  private Leds() {
    leds = new AddressableLED(8);
    buffer = new AddressableLEDBuffer(fullLength);
    leds.setLength(fullLength);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    fullSection,
                    Color.kWhite,
                    Color.kBlack,
                    breathSlowDuration,
                    Timer.getFPGATimestamp());
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      disabledColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(disabledColor);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
    }

    // Update auto state
    if (DriverStation.isEnabled()) {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getTimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Strategy priorities
    firstPriorityColor =
        renderPriority(firstPriorityLevel, firstPriorityBlocked, firstPrioritySection);
    secondPriorityColor =
        renderPriority(secondPriorityLevel, secondPriorityBlocked, secondPrioritySection);

    // Select LED mode
    solid(fullSection, Color.kBlack); // Default to off
    if (estopped) {
      solid(fullSection, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (superstructureCoast) {
        // Elevator coast alert
        solid(fullSection, Color.kWhite);
      } else if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        wave(
            new Section(
                (int) (length * ((Timer.getTimestamp() - lastEnabledTime) / autoFadeTime)), length),
            Color.kGold,
            Color.kDarkBlue,
            waveFastCycleLength,
            waveFastDuration);
      } else if (lowBatteryAlert) {
        // Low battery
        strobe(fullSection, Color.kOrangeRed, Color.kBlack, strobeDuration);
      } else if (prideLeds) {
        // Pride stripes
        stripes(
            fullSection,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
      } else if (superClimbed) {
        wave(fullSection, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      } else {
        // Default pattern
        wave(
            fullSection,
            disabledColor,
            secondaryDisabledColor,
            waveDisabledCycleLength,
            waveDisabledDuration);
      }

      // Vision disconnected alert
      if (visionDisconnected) {
        strobe(sideSection, Color.kRed, Color.kBlack, strobeDuration);
      }

    } else if (DriverStation.isAutonomous()) {
      if (characterizationMode) {
        strobe(fullSection, Color.kGold, Color.kBlack, 0.5);
      } else {
        wave(fullSection, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      }
    } else {
      solid(firstPrioritySection, firstPriorityColor);
      solid(secondPrioritySection, secondPriorityColor);

      // Auto scoring reef
      if (autoScoringReef) {
        rainbow(straightSection, rainbowCycleLength, rainbowDuration);
        solid(
            sideSection,
            switch (autoScoringLevel) {
              case L1 -> l1PriorityColor;
              case L2 -> l2PriorityColor;
              case L3 -> l3PriorityColor;
              case L4 -> l4PriorityColor;
            });

        // Super auto scoring
        if (superAutoScoring) {
          strobe(straightSection, Color.kBlack, null, rainbowStrobeDuration);
        }
      }

      // Auto scoring
      if (autoScoring) {
        rainbow(straightSection, rainbowCycleLength, rainbowDuration);
      }

      // Ready alert
      if (ready) {
        strobe(straightSection, Color.kWhite, Color.kBlue, strobeDuration);
      }

      // Coral grab alert
      if (coralGrabbed) {
        solid(straightSection, Color.kLime);
      }

      // Human player alert
      if (hpAttentionAlert) {
        strobe(straightSection, Color.kWhite, Color.kBlack, strobeDuration);
      }

      // Endgame alert
      if (endgameAlert) {
        strobe(straightSection, Color.kRed, Color.kGold, strobeDuration);
      }
    }

    // Superstructure estop alert
    if (superstructureEstopped) {
      solid(fullSection, Color.kRed);
    }

    // Update dashboard
    SmartDashboard.putString("LEDs/First Priority", firstPriorityColor.toHexString());
    SmartDashboard.putString("LEDs/Second Priority", secondPriorityColor.toHexString());

    // Update LEDs
    leds.setData(buffer);

    // Record cycle time
    LoggedTracer.record("LEDs");
  }

  private Color solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        setLED(i, color);
      }
    }
    return color;
  }

  private Color strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    if ((c1On && c1 == null) || (!c1On && c2 == null)) return null;
    return solid(section, c1On ? c1 : c2);
  }

  private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    Color color = breathCalculate(section, c1, c2, duration, timestamp);
    solid(section, color);
    return color;
  }

  private Color breathCalculate(
      Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      x %= 180.0;
      setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      setLED(i, colors.get(colorIndex));
    }
  }

  private Color renderPriority(Optional<ReefLevel> level, Boolean blocked, Section section) {
    Color primaryColor =
        level.isEmpty()
            ? Color.kBlack
            : switch (level.get()) {
              case L1 -> l1PriorityColor;
              case L2 -> l2PriorityColor;
              case L3 -> l3PriorityColor;
              case L4 -> l4PriorityColor;
            };
    if (!blocked) {
      return primaryColor;
    } else {
      return breathCalculate(
          section,
          primaryColor,
          Color.lerpRGB(primaryColor, Color.kBlack, 0.9),
          breathFastDuration,
          Timer.getTimestamp());
    }
  }

  private void setHSV(int index, int h, int s, int v) {
    setLED(index, Color.fromHSV(h, s, v));
  }

  private void setLED(int index, Color color) {
    var indices = getIndices(index);
    if (indices.getFirst() < 0) return;
    var dimmedColor =
        shouldDimSupplier.getAsBoolean()
            ? Color.lerpRGB(Color.kBlack, color, dimMultiplier)
            : color;
    buffer.setLED(indices.getFirst(), dimmedColor);
    buffer.setLED(indices.getSecond(), dimmedColor);
  }

  private static Pair<Integer, Integer> getIndices(int index) {
    if (index >= virtualSection.start() && index < virtualSection.end()) {
      return Pair.of(-1, -1);
    } else if (index >= virtualSection.end()) {
      index -= virtualSectionLength;
    }
    int a = MathUtil.clamp(index, 0, fullLength - 1);
    int b = MathUtil.clamp(fullLength - index - 1, 0, fullLength - 1);
    return Pair.of(a, b);
  }

  private record Section(int start, int end) {}
}
