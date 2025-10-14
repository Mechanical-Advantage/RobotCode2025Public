// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.Optional;

public class CanivoreReader {
  private final CANBus canBus;
  private final Thread thread;
  private Optional<CANBusStatus> status = Optional.empty();

  public CanivoreReader(String canBusName) {
    canBus = new CANBus(canBusName);
    thread =
        new Thread(
            () -> {
              while (true) {
                var statusTemp = Optional.of(canBus.getStatus());
                synchronized (this) {
                  status = statusTemp;
                }
                try {
                  Thread.sleep(400); // Match RIO CAN sampling
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              }
            });
    thread.setName("CanivoreReader");
    thread.start();
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }
}
