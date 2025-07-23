// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;

import java.util.Optional;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.TunableNumber;

public final class Constants {

  public static boolean TUNING_MODE = true;

  public static Optional<DriverStation.Alliance> ALLIANCE_COLOR = DriverStation.getAlliance();

  public static final Mass ROBOT_MASS = Pound.of(140);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(4.563);

  public static String IS_RED = "N/A"; // FIXME: MAKE AUTO UPDATE ISRED
}
