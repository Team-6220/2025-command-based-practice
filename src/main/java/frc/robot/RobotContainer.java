// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.IntakeIdleCmd;
import frc.robot.Commands.ManuelArmAndIntakeCmd;
import frc.robot.Commands.TankDriveCmd;
import frc.robot.Subsystems.ArmSubSystem;
import frc.robot.Subsystems.DriveTrainSubsystem;

public class RobotContainer {

  public DriveTrainSubsystem driveTrainSubsystem = DriveTrainSubsystem.getInstance();
  public ArmSubSystem armSubSystem = ArmSubSystem.getInstance();

  public Joystick joystick = new Joystick(1);
  public XboxController xboxController = new XboxController(0);


  private final Trigger resetEncoder = new Trigger(()-> joystick.getRawButton(2));

  public RobotContainer() {
    driveTrainSubsystem.setDefaultCommand(new TankDriveCmd(xboxController));
    armSubSystem.setDefaultCommand(new ManuelArmAndIntakeCmd(() -> joystick.getY(), ()-> joystick.getRawAxis(2)));
    // armSubSystem.setDefaultCommand(new IntakeIdleCmd());
    
    configureBindings();
  }

  private void configureBindings() {
    resetEncoder.onTrue(new InstantCommand(() -> armSubSystem.resetRelativeEncoder()));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
