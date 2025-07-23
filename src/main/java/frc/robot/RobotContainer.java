// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.ManuelArmCmd;
import frc.robot.Commands.TankDriveCmd;
import frc.robot.Subsystems.ArmSubSystem;
import frc.robot.Subsystems.DriveTrainSubsystem;

public class RobotContainer {

  public DriveTrainSubsystem driveTrainSubsystem = DriveTrainSubsystem.getInstance();
  public ArmSubSystem armSubSystem = ArmSubSystem.getInstance();

  public XboxController joystick = new XboxController(0);

  // private final Trigger spinToDegree = new Trigger(()-> joystick.getAButton());
  // private final Trigger spinTo90 = new Trigger(()-> joystick.getBButton());

  public RobotContainer() {
    driveTrainSubsystem.setDefaultCommand(new TankDriveCmd((joystick)));
    armSubSystem.setDefaultCommand(new ManuelArmCmd(() -> joystick.getLeftY()));

    configureBindings();
  }

  private void configureBindings() {
    // spinToDegree.onTrue(new SpinToDegreeCmd());
    // spinTo90.onTrue(new MoveWristTo90Cmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
