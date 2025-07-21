// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DrivetrainConstants;

public class DriveTrainSubsystem extends SubsystemBase{
  private static DriveTrainSubsystem INSTANCE = null;

  private String tableKey = "DriveTrain_";
  TalonSRXConfiguration driveMotorConfig = new TalonSRXConfiguration();
  WPI_TalonSRX leftMain = new WPI_TalonSRX(DrivetrainConstants.LEFTMAIN_ID);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(DrivetrainConstants.LEFTFOLLOWER_ID);

  WPI_TalonSRX rightMain = new WPI_TalonSRX(DrivetrainConstants.RIGHTMAIN_ID);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(DrivetrainConstants.RIGHTFOLLOWER_ID);


  public DriveTrainSubsystem() {
    leftFollower.follow(leftMain);
    rightFollower.follow(rightMain);

    driveMotorConfig.continuousCurrentLimit = DrivetrainConstants.DRIVETRAINCURRENTLIMIT_AMPS;
    leftMain.configAllSettings(driveMotorConfig);
    leftMain.configAllSettings(driveMotorConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(tableKey +"left main temp", leftMain.getTemperature());
    SmartDashboard.putNumber(tableKey +"left follower temp", leftFollower.getTemperature());
    SmartDashboard.putNumber(tableKey +"right main temp", rightMain.getTemperature());
    SmartDashboard.putNumber(tableKey +"right follower temp", rightFollower.getTemperature());

    // This method will be called once per scheduler run
  }
  public void tankDrive(double left, double right)
  {
    leftMain.set(left);
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
