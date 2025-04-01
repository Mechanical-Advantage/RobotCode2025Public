// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/superstructure/SuperstructureConstants.h"

#include <frc/math/units/units.h>

#include "util/LoggedTunableNumber.h"

LoggedTunableNumber SuperstructureConstants::throwHeight = LoggedTunableNumber(
    "Superstructure/Throw/Height", SuperstructureConstants::elevatorMaxTravel);
LoggedTunableNumber SuperstructureConstants::throwVelocity =
    LoggedTunableNumber("Superstructure/Throw/Velocity", 3.0);