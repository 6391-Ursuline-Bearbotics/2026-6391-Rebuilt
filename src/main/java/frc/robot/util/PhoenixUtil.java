// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    if (maxAttempts <= 0) {
      throw new IllegalArgumentException("maxAttempts must be positive");
    }

    StatusCode error = null;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error.isOK()) {
        return error;
      }
    }

    DriverStation.reportError(
        "Phoenix command failed after " + maxAttempts + " attempts: " + error, false);
    return error;
  }
}
