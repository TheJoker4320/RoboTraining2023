// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PidConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private double overallDistance;
  private double Xdisplacement = 0;
  private double Ydisplacement = 0;
  private double Zdisplacement = 0;

  private double overallSpeed = 0;
  private double Xacceleration = 0;
  private double Yvelocity = 0;
  private double Zvelocity = 0;
  private final WPI_VictorSPX rightMotor;
  private final WPI_VictorSPX leftMotor;
  private final WPI_VictorSPX rightSlaveMotor;
  private final WPI_VictorSPX leftSlaveMotor;

  private final AHRS navXGyro = new AHRS();

  private Encoder rightEncoder;
  private Encoder leftEncoder;

  private final DifferentialDrive differentialDrive;
  private static Chassis instance;

  private final PIDController pidController = new PIDController(Constants.PidConstants.Kp, Constants.PidConstants.Ki, Constants.PidConstants.Kd);

  public static Chassis getInstance() {
    if (instance == null) instance = new Chassis();
    return instance;
  }

  private Chassis() {
    navXGyro.resetDisplacement();
    pidController.reset();
    pidController.setTolerance(PidConstants.TOLERANCE);

    rightMotor = new WPI_VictorSPX(ChassisConstants.RIGHT_MASTER_MOTOR_PORT);
    rightMotor.configFactoryDefault();
    
    rightSlaveMotor = new WPI_VictorSPX(ChassisConstants.RIGHT_SLAVE_MOTOR_PORT);
    rightSlaveMotor.configFactoryDefault();
    rightSlaveMotor.follow(rightMotor);

    leftMotor = new WPI_VictorSPX(ChassisConstants.LEFT_MASTER_MOTOR_PORT);
    leftMotor.configFactoryDefault();

    leftSlaveMotor = new WPI_VictorSPX(ChassisConstants.LEFT_SLAVE_MOTOR_PORT);
    leftSlaveMotor.configFactoryDefault();
    leftSlaveMotor.follow(leftMotor);
    
    leftEncoder=new Encoder(ChassisConstants.LEFT_ENCODER_SCORE[0], ChassisConstants.LEFT_ENCODER_SCORE[1]);
    rightEncoder=new Encoder(ChassisConstants.RIGHT_ENCODER_SCORE[0], ChassisConstants.RIGHT_ENCODER_SCORE[1]);

    leftEncoder.setDistancePerPulse(Constants.PidConstants.RATIO_TICKS_TO_METERS);
    rightEncoder.setDistancePerPulse(Constants.PidConstants.RATIO_TICKS_TO_METERS);
    
    rightEncoder.reset();
    leftEncoder.reset();

    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
  }

  public void drive(final double forwardSpeed, final double rotationSpeed) {
    /*Xacceleration = navXGyro.getWorldLinearAccelX();
    Xdisplacement = navXGyro.getDisplacementX();
    Ydisplacement = navXGyro.getDisplacementY();
    Zdisplacement = navXGyro.getDisplacementZ();
    overallDistance = Math.
    overallDistance = Math.sqrt(Xdisplacement * Xdisplacement + Ydisplacement * Ydisplacement + Zdisplacement * Zdisplacement);
    SmartDashboard.putNumber("NavX distantce", overallDistance);*/
    SmartDashboard.putNumber("Encoder Value Right", rightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value Left", leftEncoder.getDistance());;
    differentialDrive.arcadeDrive(-rotationSpeed, -forwardSpeed);
  }

  public void autonomouseDrive() {
    //differentialDrive.arcadeDrive(0,(pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.SET_POINT)));
    SmartDashboard.putNumber("Tolerance", pidController.getPositionTolerance());
    SmartDashboard.putNumber("PID Value Right", pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.SET_POINT));
    SmartDashboard.putNumber("PID Value Left", pidController.calculate(rightEncoder.getDistance(), Constants.PidConstants.SET_POINT));;
    leftMotor.set(-1 * (pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.SET_POINT)));
    rightMotor.set( pidController.calculate(rightEncoder.getDistance(), Constants.PidConstants.SET_POINT));
  }

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public Encoder getLeftEncoder(){
    return leftEncoder;
  }
  
  public Encoder getRightEncoder(){
    return rightEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
