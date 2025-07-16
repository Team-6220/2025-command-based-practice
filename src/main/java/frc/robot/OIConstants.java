// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class OIConstants {
    public static final double kDeadband = 0.2;
    public static double normalizeAxis(double value) {
        if (Math.abs(value) < kDeadband) {
          return 0;
        }
      
        double x = (Math.abs(value) - kDeadband) / (1.0 - kDeadband); // map to [0,1]
        x = Math.min(Math.max(x, 0), 1); // clamp just in case
      
        // Blend cubic and linear response
        double a = 0.5;
        double output = a * Math.pow(x, 3) + (1 - a) * x;
      
        // Restore sign
        return Math.copySign(output, value);
      }
}
