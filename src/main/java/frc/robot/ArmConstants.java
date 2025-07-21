// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class ArmConstants {
      public static final int ArmMotorMainID = 18;
      public static final int ArmMotorFollowerID = 18;

  public static final boolean mainMotorInverted = false;
  public static final boolean followerMotorInverted = false;
  
  public static final IdleMode ArmIdleMode = IdleMode.kBrake;

  public static final boolean encoderInverted = true;
  /*on branch tune_lower_intake PID&FF start (not really tuned) */
  // public static final int stallLimit = 10;
  // public static final int freeLimit = 10;

  // public static final double kP = 0.00, kI = 0, kD = 0, izone = 2, tolerance = .5;
  // public static final double kS = 0, kG = .19, kV = .53, kA = 0;
  // public static final double allowedClosedLoopError = 0.5;
  // public static final double maxAcceleration = 720, maxVelocity = 360;//Accelaration is in
  // units of RPM per Second (RPM/s) & Maximum Velocity is in units of Revolutions per Minute
  // (RPM)
  // public static final double ArmMaxDegrees = 87, ArmMinDegrees = -144;
  /*on branch tune_lower_intake PID&FF end (not really tuned) */
  public static final double intakeSetpointDegree = 0;
  public static final double outtakeSetpointDegree = 90;
  public static final double coralStation = 35.81813;

  public static final int stallLimit = 25;
  public static final int freeLimit = 5;

  public static final double kP = 0.026, kI = 0.02, kD = 0.0001, izone = 05, tolerance = 1.0;
  public static final double kS = 0, kG = 0, kV = 0, kA = 0;
  public static final double allowedClosedLoopError = 0.5;
  public static final double maxAcceleration = 720,
      maxVelocity =
          360; // Accelaration is in units of RPM per Second (RPM/s) & Maximum Velocity is in
  // units of Revolutions per Minute (RPM)
  public static final double ArmMaxDegrees = 120, ArmMinDegrees = 0;
  /*on branch scrimage v2 PID&FF end (not really tuned) */
}
