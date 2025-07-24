// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class ArmConstants {
  public static final int MAIN_MOTOR_ID = 18;
  public static final int FOLLOWER_MOTOR_ID = 19;

  public static final boolean MAIN_MOTOR_INVERTED = false;
  public static final boolean FOLLOWER_MOTOR_INVERTED = false;
  
  public static final IdleMode ARM_IDLEMODE = IdleMode.kBrake;

  public static final boolean ENCODER_INVERTED = true;


  public static final double INTAKE_SETPOINT_DEGREES = 0;
  public static final double OUTTAKE_SETPOINT_DEGREES = 90;
  public static final double IDLE_SETPOINT_DEGREES = 100;

  public static final int STALL_LIMIT_AMP = 25;
  public static final int FREE_LIMIT_AMP = 5;

  public static final double kP = 0.026, kI = 0.02, kD = 0.0001, izone = 05, tolerance = 1.0;
  public static final double kS = 0, kG = 0, kV = 0, kA = 0;
  public static final double ALLOWED_CLOSED_LOOP_ERROR = 0.5;
  public static final double MAX_ACCELERATION = 720,
      MAX_VELOCITY =
          360; // Accelaration is in units of RPM per Second (RPM/s) & Maximum Velocity is in
  // units of Revolutions per Minute (RPM)
  public static final double ARM_MAX_DEGREES = 120, ARM_MIN_DEGREES = 0;

  public static final NeutralModeValue INTAKENEU_NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final int INTAKE_MOTOR_ID = 15;
  public static final double EJECT_SPEED_PERCENT = -.1;
  public static final double INTAKE_SPEED_PERCENT = .1;

  public static final boolean CURRENT_LIMIT_ENABLED = true;
  public static final double NORMAL_MAX_CURRENT = 20;
  public static final double ABSOLUTE_MAX_CURRENT = 5;
  public static final double ABSOLUTE_MAX_CURRENT_TIME = 1;
}
