// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PidConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static double distantceTraveled;

  
  private double Xdisplacement = 0;
  private double Ydisplacement = 0;
  private double Zdisplacement = 0;

  private double overallSpeed = 0;
  private double Xacceleration = 0;
  private double Yvelocity = 0;
  private double Zvelocity = 0;
  private final WPI_TalonSRX rightMotor;
  private final WPI_TalonSRX leftMotor;
  private final WPI_TalonSRX rightSlaveMotor;
  private final WPI_TalonSRX leftSlaveMotor;

  private final MotorControllerGroup righMotorControllerGroup;
  private final MotorControllerGroup leftMotorControllerGroup;

  public static int counter;
  public final AHRS navXGyro = new AHRS();

  private Encoder rightEncoder;
  private Encoder leftEncoder;

  private final DifferentialDrive differentialDrive;
  private final DifferentialDriveOdometry differentialDriveOdometry;
  
  private static Chassis instance;
  private final PIDController pidController = new PIDController(Constants.PidConstants.Kp, Constants.PidConstants.Ki, Constants.PidConstants.Kd);
  public static Timer timer;
  public static double currentAccelTimeStamp;
  public static double prevAccelTimeStamp;
  public static double currentVelocityTimeStamp;
  public static double prevVelocityTimeStamp;
  public static double currentAccel;
  public static double prevAccel;
  public static double currentVelocity;
  public static double prevVelocity;

  public double overallDistance;

  public static Chassis getInstance() {
    if (instance == null) instance = new Chassis();
    return instance;
  }

  private Chassis() {
    timer = new Timer();
    timer.start();
    overallDistance = 0;
    prevAccel = 0;
    prevAccelTimeStamp = 0;
    prevVelocity = 0;
    prevVelocityTimeStamp = 0;

    //--------------------------------
    counter = 0;
    pidController.setTolerance(PidConstants.TOLERANCE);

    rightMotor = new WPI_TalonSRX(ChassisConstants.RIGHT_MASTER_MOTOR_PORT);
    rightMotor.configFactoryDefault();
    
    rightSlaveMotor = new WPI_TalonSRX(ChassisConstants.RIGHT_SLAVE_MOTOR_PORT);
    rightSlaveMotor.configFactoryDefault();
    rightSlaveMotor.follow(rightMotor);

    leftMotor = new WPI_TalonSRX(ChassisConstants.LEFT_MASTER_MOTOR_PORT);
    leftMotor.configFactoryDefault();

    leftSlaveMotor = new WPI_TalonSRX(ChassisConstants.LEFT_SLAVE_MOTOR_PORT);
    leftSlaveMotor.configFactoryDefault();
    leftSlaveMotor.follow(leftMotor);
    
    leftEncoder=new Encoder(ChassisConstants.LEFT_ENCODER_SCORE[0], ChassisConstants.LEFT_ENCODER_SCORE[1]);
    rightEncoder=new Encoder(ChassisConstants.RIGHT_ENCODER_SCORE[0], ChassisConstants.RIGHT_ENCODER_SCORE[1]);

    leftEncoder.setDistancePerPulse(Constants.PidConstants.RATIO_TICKS_TO_METERS);
    rightEncoder.setDistancePerPulse(Constants.PidConstants.RATIO_TICKS_TO_METERS);

    righMotorControllerGroup = new MotorControllerGroup(rightMotor, rightSlaveMotor);
    leftMotorControllerGroup = new MotorControllerGroup(leftMotor, leftSlaveMotor);
    
    differentialDrive = new DifferentialDrive(leftMotorControllerGroup, righMotorControllerGroup);

    navXGyro.resetDisplacement();
    navXGyro.reset();
    navXGyro.calibrate();

    pidController.reset();
    rightEncoder.reset();
    leftEncoder.reset();

    differentialDriveOdometry = new DifferentialDriveOdometry(navXGyro.getRotation2d(), 0, 0);
    differentialDriveOdometry.resetPosition(navXGyro.getRotation2d(), 0, 0, new Pose2d());
  }

  public void drive(final double forwardSpeed, final double rotationSpeed) {
    /*Xacceleration = navXGyro.getWorldLinearAccelX();
    Xdisplacement = navXGyro.getDisplacementX();
    Ydisplacement = navXGyro.getDisplacementY();
    Zdisplacement = navXGyro.getDisplacementZ();
    overallDistance = Math.
    overallDistance = Math.sqrt(Xdisplacement * Xdisplacement + Ydisplacement * Ydisplacement + Zdisplacement * Zdisplacement);
    SmartDashboard.putNumber("NavX distantce", overallDistance);*/
    SmartDashboard.putNumber("Encoder Value Right", rightEncoder.getDistance() / rightEncoder.getDistancePerPulse());
    SmartDashboard.putNumber("Encoder Value Left", leftEncoder.getDistance() / leftEncoder.getDistancePerPulse());
    SmartDashboard.putNumber("Encoder Value Right - Meters",  -1 * rightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value Left - Meters",leftEncoder.getDistance());
    differentialDrive.arcadeDrive(-rotationSpeed, -forwardSpeed);
  }

  public void autonomouseDrive() {
    //differentialDrive.arcadeDrive(0,(pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.SET_POINT)));
    SmartDashboard.putNumber("Tolerance", pidController.getPositionTolerance());
    SmartDashboard.putNumber("PID Value Right", pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.SET_POINT));
    SmartDashboard.putNumber("PID Value Left", pidController.calculate(rightEncoder.getDistance(), Constants.PidConstants.SET_POINT));;
    leftMotorControllerGroup.set(1 * (pidController.calculate(-1 * leftEncoder.getDistance(), Constants.PidConstants.SET_POINT)));
    righMotorControllerGroup.set(-1 * (pidController.calculate(rightEncoder.getDistance(), Constants.PidConstants.SET_POINT)));
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

  //-------------------------------------------------------
  public AHRS getGyro() {
    return navXGyro;
  }
  public double getRightEncoderPosition() {
    return rightEncoder.getDistance();
  }
  public double getLeftEncoderPosition() {
    return leftEncoder.getDistance();
  }
  public double getRightEncoderVelocity() {
    return rightEncoder.getRate();
  }
  public double getLeftEncoderVelocity() {
    return leftEncoder.getRate();
  }
  public double getAverageEncoderDistantce() {
    return (getLeftEncoderPosition() + getRightEncoderPosition())/2;
  }
  public double getTurnRate() {
    return -navXGyro.getRate();
  }
  public double getHeading() {
    return navXGyro.getRotation2d().getDegrees();
  }
  public Pose2d getPose() {
    return differentialDriveOdometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    differentialDriveOdometry.resetPosition(navXGyro.getRotation2d(), 0, 0, pose);
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  public void tankDriveWolts(double leftVolts, double rightVolts) {
    counter++;
    SmartDashboard.putNumber("This was called - times", counter);
    righMotorControllerGroup.setVoltage(rightVolts);
    leftMotorControllerGroup.setVoltage(-leftVolts);
    /*rightMotor.configVoltageCompSaturation(rightVolts);
    leftMotor.configVoltageCompSaturation(leftVolts);
    rightMotor.enableVoltageCompensation(true);
    leftMotor.enableVoltageCompensation(true);*/
    differentialDrive.feed();
  }
  //-------------------------------------------------------
  public void stopDriving() {
    righMotorControllerGroup.set(0);
    leftMotorControllerGroup.set(0);
  }
  @Override
  public void periodic() {
    differentialDriveOdometry.update(navXGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("Gyro Heading", getHeading());

    currentAccelTimeStamp = timer.get();
    currentAccel = navXGyro.getWorldLinearAccelX();

    currentVelocity = (currentAccelTimeStamp - prevAccelTimeStamp) * ((currentAccel + prevAccel)/2);

    currentVelocityTimeStamp = (currentAccelTimeStamp + prevAccelTimeStamp)/2;
    overallDistance += (currentVelocityTimeStamp - prevVelocityTimeStamp) * ((currentVelocity + prevVelocity)/2);

    prevAccel = currentAccel;
    prevAccelTimeStamp = currentAccelTimeStamp;
    prevVelocity = currentVelocity;
    prevVelocityTimeStamp = currentVelocityTimeStamp;

    SmartDashboard.putNumber("Distance traveled", overallDistance);
    SmartDashboard.putNumber("acceleration", currentAccel);
    SmartDashboard.putNumber("velocity", currentVelocity);
  }
}
