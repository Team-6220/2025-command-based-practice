// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ManuelSpinMotorCmd;
import frc.robot.Commands.SpinToDegreeCmd;
import frc.robot.Subsystems.MotorSubsystem;
import frc.robot.Subsystems.V2_SparkMaxWristSubsystem;

public class RobotContainer {

  public V2_SparkMaxWristSubsystem wristPIDTest = V2_SparkMaxWristSubsystem.getInstance();

  public XboxController joystick = new XboxController(0);

  private final Trigger spinToDegree = new Trigger(()-> joystick.getAButton());

  public RobotContainer() {
    wristPIDTest.setDefaultCommand(new ManuelSpinMotorCmd(joystick));
    configureBindings();
  }

  private void configureBindings() {
    spinToDegree.onTrue(new SpinToDegreeCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
