// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Subsystems;

// import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class MotorSubsystem extends SubsystemBase {
//   /** Creates a new motorSubsystem. */

//   private static MotorSubsystem INSTANCE = null;

//   WPI_VictorSPX testMotor;
//   VictorSPXConfiguration victorSPXConfiguration;

//   private double percentInDecimal;
//   public MotorSubsystem() {
//     testMotor = new WPI_VictorSPX(17);
//     victorSPXConfiguration = new VictorSPXConfiguration();
//     victorSPXConfiguration.peakOutputForward = 0.1;
//     victorSPXConfiguration.peakOutputReverse = 0.1;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("output", percentInDecimal);
//   }
//   public void spinMotorPercentage(double percentInDecimal){
//     this.percentInDecimal = percentInDecimal;
//     testMotor.set(percentInDecimal);
//   }

//   public static synchronized MotorSubsystem getInstance() {
//     if (INSTANCE == null) {
//       INSTANCE = new MotorSubsystem();
//     }
//     return INSTANCE;
//   }
// }
