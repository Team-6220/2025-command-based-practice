// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DrivetrainConstants;

public class DriveTrainSubsystem extends SubsystemBase{
  private static DriveTrainSubsystem INSTANCE = null;

  private String tableKey = "DriveTrain_";
  VictorSPXConfiguration leftDriveMotorConfig = new VictorSPXConfiguration();
  VictorSPXConfiguration rightDriveMotorConfig = new VictorSPXConfiguration();
  WPI_VictorSPX leftMain = new WPI_VictorSPX(DrivetrainConstants.LEFTMAIN_ID);
  WPI_VictorSPX leftFollower = new WPI_VictorSPX(DrivetrainConstants.LEFTFOLLOWER_ID);

  WPI_VictorSPX rightMain = new WPI_VictorSPX(DrivetrainConstants.RIGHTMAIN_ID);
  WPI_VictorSPX rightFollower = new WPI_VictorSPX(DrivetrainConstants.RIGHTFOLLOWER_ID);


  public DriveTrainSubsystem() {
    leftFollower.follow(leftMain);
    rightFollower.follow(rightMain);

    // driveMotorConfig. = DrivetrainConstants.DRIVETRAINCURRENTLIMIT_AMPS;
    // leftMain.configAllSettings(driveMotorConfig);
    // rightMain.configAllSettings(driveMotorConfig);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber(tableKey +"left main temp", leftMain.getTemperature());
    // SmartDashboard.putNumber(tableKey +"left follower temp", leftFollower.getTemperature());
    // SmartDashboard.putNumber(tableKey +"left follower2 temp", leftFollower2.getTemperature());
    // SmartDashboard.putNumber(tableKey +"right main temp", rightMain.getTemperature());
    // SmartDashboard.putNumber(tableKey +"right follower temp", rightFollower.getTemperature());
    // SmartDashboard.putNumber(tableKey +"right follower2 temp", rightFollower2.getTemperature());

    // This method will be called once per scheduler run
  }
  public void tankDrive(double left, double right)
  {
    System.out.println("left:" + left + "right" + right);
    left = left > 0.95 ? 0.95 : left < -0.95 ? -0.95 : left;
    right = right > 0.95 ? 0.95 : right < -0.95 ? -0.95 : right;
    leftMain.set(-left);
    rightMain.set(right);
  }

  /**
   * Accesses the static instance of the subsystem singleton
   *
   * @return ArmSubsystem Singleton Instance
   */
  public static synchronized DriveTrainSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new DriveTrainSubsystem();
    }
    return INSTANCE;
  }
}
