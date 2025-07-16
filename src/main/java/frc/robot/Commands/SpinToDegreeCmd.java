// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.TunableNumber;
import frc.robot.Subsystems.V2_SparkMaxWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinToDegreeCmd extends Command {
  /** Creates a new SpinToDegreeCmd. */
  V2_SparkMaxWristSubsystem wristSubsystem = V2_SparkMaxWristSubsystem.getInstance();

  TunableNumber wristGoal = new TunableNumber("set wrist goal degrees here", 0);
  public SpinToDegreeCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(getRequirements());
  }

  @Override
  public Set<Subsystem> getRequirements() {
      Set<Subsystem> commandRequirements = super.getRequirements();
      commandRequirements.add(wristSubsystem);
      return commandRequirements;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setGoal(wristGoal.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(wristGoal.hasChanged()){
      wristSubsystem.setGoal(wristGoal.get());
    }
    wristSubsystem.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
