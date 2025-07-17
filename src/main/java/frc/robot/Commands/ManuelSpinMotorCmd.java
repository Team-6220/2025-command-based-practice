// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OIConstants;
import frc.robot.Subsystems.MotorSubsystem;
import frc.robot.Subsystems.V2_SparkMaxWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManuelSpinMotorCmd extends Command {
  /** Creates a new ManuelSpinMotorCmd. */
  V2_SparkMaxWristSubsystem wristSubsystem = V2_SparkMaxWristSubsystem.getInstance();

  XboxController joystick;
  public ManuelSpinMotorCmd(XboxController joystick) {
    this.joystick = joystick;
    addRequirements(wristSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    wristSubsystem.simpleDrive(OIConstants.normalizeAxis(joystick.getLeftY()));;
=======
    wristSubsystem.simpleDrive(OIConstants.normalizeAxis(joystick.getY())*2);
>>>>>>> 0ca1b3919823e9aa1de0176e8f88cd27d3aecc35
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.simpleDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
