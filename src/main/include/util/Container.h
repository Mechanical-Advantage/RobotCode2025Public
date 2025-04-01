// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

template <typename T> class Container {
public:
  T value;

  Container() {}
  Container(T initialValue) : value(initialValue) {}
};