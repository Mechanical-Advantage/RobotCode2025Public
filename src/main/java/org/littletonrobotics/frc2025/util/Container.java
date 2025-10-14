// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

public class Container<T> {
  public T value;

  public Container() {}

  public Container(T initialValue) {
    value = initialValue;
  }
}
