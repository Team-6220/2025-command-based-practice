// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ManuelSpinMotorCmd;
import frc.robot.Commands.SpinToDegreeCmd;
import frc.robot.Subsystems.MotorSubsystem;

public class RobotContainer {

  public MotorSubsystem motorSubsystem = MotorSubsystem.getInstance();

  public Joystick joystick = new Joystick(0);

  private final Trigger spinToDegree = new Trigger(()-> joystick.getRawButton(1));

  public RobotContainer() {
    motorSubsystem.setDefaultCommand(new ManuelSpinMotorCmd(joystick));
    configureBindings();
  }

  private void configureBindings() {
    spinToDegree.onTrue(new SpinToDegreeCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
