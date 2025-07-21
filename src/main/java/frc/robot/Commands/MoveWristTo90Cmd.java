// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.V2_SparkMaxWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWristTo90Cmd extends Command {
  V2_SparkMaxWristSubsystem wrist = V2_SparkMaxWristSubsystem.getInstance();
  /** Creates a new MoveWristTo90Cmd. */
  public MoveWristTo90Cmd() {
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setGoal(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
