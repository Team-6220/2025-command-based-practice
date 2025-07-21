// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//MAKE SURE YOU SET UP THE CIM MODE IN REV

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.ArmConstants;

public class ArmSubSystem extends SubsystemBase {
  /** Creates a new V2_SparkMaxArmSubsystem. */
  private static ArmSubSystem INSTANCE = null;

  private final TunableNumber ArmKp =
      new TunableNumber("Arm kP", ArmConstants.kP); // TODO: match/make to constant.java
  private final TunableNumber ArmKi = new TunableNumber("Arm kI", ArmConstants.kI);
  private final TunableNumber ArmKd = new TunableNumber("Arm kD", ArmConstants.kD);
  private final TunableNumber ArmKg = new TunableNumber("Arm kG", ArmConstants.kG);
  private final TunableNumber ArmKv = new TunableNumber("Arm kV", ArmConstants.kV);
  private final TunableNumber ArmKs = new TunableNumber("Arm kS", ArmConstants.intakeSetpointDegree);
  private final TunableNumber ArmIZone =
      new TunableNumber("Arm izone", ArmConstants.izone); // default 3
  private final TunableNumber ArmTolerance =
      new TunableNumber("Arm tolerance", ArmConstants.tolerance); // default 1.5

  private final TunableNumber ArmMaxVel =
      new TunableNumber("Arm max vel", ArmConstants.maxVelocity);
  private final TunableNumber ArmMaxAccel =
      new TunableNumber("Arm max accel", ArmConstants.maxAcceleration);

  private final SparkMax armMotorMain;
  private final SparkMax armMotorFollower;
  private SparkMaxConfig mainArmMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig followerArmMotorConfig = new SparkMaxConfig();



  private final String tableKey = "Arm_";

  /*
    Arm PID & FF stuff
    see:
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-Arm.html
    for more detail
  */
  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;

  private final AbsoluteEncoder ArmEncoder;

  public ArmSubSystem() {
    armMotorMain = new SparkMax(ArmConstants.ArmMotorMainID, MotorType.kBrushless);
    armMotorFollower = new SparkMax(ArmConstants.ArmMotorFollowerID, MotorType.kBrushless);

    mainArmMotorConfig
        .inverted(ArmConstants.mainMotorInverted)
        .smartCurrentLimit(ArmConstants.stallLimit, ArmConstants.freeLimit)
        .idleMode(ArmConstants.ArmIdleMode);
    mainArmMotorConfig
    .absoluteEncoder
    .inverted(ArmConstants.encoderInverted)
    .positionConversionFactor(
        360) // basically this turns the encoder reading from radians to degrees
        .zeroOffset(0.7785330) //TODO: change this
        .zeroCentered(true);
            
    followerArmMotorConfig
        .inverted(ArmConstants.followerMotorInverted)
        .smartCurrentLimit(ArmConstants.stallLimit, ArmConstants.freeLimit)
        .idleMode(ArmConstants.ArmIdleMode);
    
    // ArmMotorConfig.absoluteEncoder.zeroOffset(.2);//Don't know if we need this

    armMotorMain.configure(
        mainArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armMotorFollower.configure(
        followerArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());

    m_Controller =
        new ProfiledPIDController(ArmKp.get(), ArmKi.get(), ArmKd.get(), m_Constraints);

    m_Feedforward = new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get());
    m_Controller.setIZone(ArmIZone.get()); // not sure if we need this
    m_Controller.setTolerance(ArmTolerance.get()); // default 1.5
    // m_Controller.setGoal(
    //  90); // cuz in manuel elevator we're calling Arm drive to goal to maintain it's position
    

    ArmEncoder = armMotorMain.getAbsoluteEncoder();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(tableKey + "Position", getArmPosition());
    SmartDashboard.putBoolean(tableKey + "atGoal", ArmAtGoal());
    SmartDashboard.putNumber("manuel output", 0);


    if (ArmKp.hasChanged() || ArmKi.hasChanged() || ArmKd.hasChanged()) {
      System.out.println("pid");
      m_Controller.setPID(ArmKp.get(), ArmKi.get(), ArmKd.get());
    }

    if (ArmKs.hasChanged() || ArmKg.hasChanged() || ArmKv.hasChanged()) {
      System.out.println(
          "update w/ new FF value: "
              + "kg: "
              + ArmKg.get()
              + ", ks"
              + ArmKs.get()
              + ", kv"
              + ArmKv.get());
      m_Feedforward = new ArmFeedforward(ArmKs.get(), ArmKg.get(), ArmKv.get());
    }

    if (ArmMaxVel.hasChanged() || ArmMaxAccel.hasChanged()) {
      m_Constraints = new TrapezoidProfile.Constraints(ArmMaxVel.get(), ArmMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
    }

    if (ArmIZone.hasChanged()) {
      m_Controller.setIZone(ArmIZone.get());
    }

    if (ArmTolerance.hasChanged()) {
      m_Controller.setTolerance(ArmTolerance.get());
    }
  }

  public void setGoal(double goal) {
    resetPID();

    if (goal > ArmConstants.ArmMaxDegrees) {
      goal = ArmConstants.ArmMaxDegrees;
    }

    if (goal < ArmConstants.ArmMinDegrees) {
      goal = ArmConstants.ArmMinDegrees;
    }

    m_Controller.setGoal(goal);
    System.out.println("Set new Arm goal: " + goal);
  }

  public void driveToGoal() {
    PIDOutput = m_Controller.calculate(getArmPosition());

    feedForwardOutput =
    m_Feedforward.calculate(
        m_Controller.getSetpoint().position * Math.PI / 180,
        m_Controller.getSetpoint().velocity * Math.PI / 180);
    double calculatedSpeed = PIDOutput + feedForwardOutput;

    SmartDashboard.putNumber("Arm Goal", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber("Wrst FF output", feedForwardOutput);
    SmartDashboard.putNumber("Arm PID out", PIDOutput);
    SmartDashboard.putNumber("Arm overall output", calculatedSpeed);
    armMotorMain.setVoltage(calculatedSpeed);
}

public void resetPID() {
    m_Controller.reset(getArmPosition());
}

/** Raw encoder value subtracted by the offset at zero */
public double getArmPosition() {
    double ArmPosition = ArmEncoder.getPosition();
    return ArmPosition;
  }
  
  /** Driving in voltage */
  public void voltsDrive(double voltageOutput) {
      SmartDashboard.putNumber("manuel output (volts)", voltageOutput);
      SmartDashboard.putNumber("manuel output (percent)", -99999);
      armMotorMain.setVoltage(voltageOutput);
    }

    public void percentDrive(double percentOutput)
    {
        SmartDashboard.putNumber("manuel output (volts)", -99999);
        SmartDashboard.putNumber("manuel output (percent)", percentOutput);
        percentOutput = percentOutput > 1 ? 1 : percentOutput < -1 ? -1 : percentOutput;
        armMotorMain.set(percentOutput);
    }

    public boolean ArmAtGoal() {
    return m_Controller.atGoal();
  }

  public double getGoalPosition() {
    return m_Controller.getGoal().position;
  }

  public void stop() {
    armMotorMain.setVoltage(0);
  }

  /**
   * Accesses the static instance of the ArmSubsystem singleton
   *
   * @return ArmSubsystem Singleton Instance
   */
  public static synchronized ArmSubSystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ArmSubSystem();
    }
    return INSTANCE;
  }
}
