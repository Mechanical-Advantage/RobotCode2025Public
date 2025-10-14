// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import edu.wpi.first.wpilibj.Threads;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/** Dummy log receiver used to adjust the thread priority of the log receiver thread. */
public class DummyLogReceiver implements LogDataReceiver {
  @Override
  public void start() {
    // DO NOT COPY UNLESS YOU UNDERSTAND THE CONSEQUENCES
    // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template#real-time-thread-priority
    Threads.setCurrentThreadPriority(true, 1);
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {}
}
