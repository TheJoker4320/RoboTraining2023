// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_VictorSPX rightMotor;
  private final WPI_VictorSPX leftMotor;
  private final WPI_VictorSPX rightSlaveMotor;
  private final WPI_VictorSPX leftSlaveMotor;

  private Encoder rightEncoder;
  private Encoder leftEncoder;

  private final DifferentialDrive differentialDrive;
  private static Chassis instance;

  private final PIDController pidController = new PIDController(Constants.PidConstants.kp, Constants.PidConstants.ki, Constants.PidConstants.kd);

  public static Chassis getInstance() {
    if (instance == null) instance = new Chassis();
    return instance;
  }

  private Chassis() {
    pidController.reset();
    pidController.setTolerance(5, 10);

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

    resetEncoders();


    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
  }

  public void drive(final double forwardSpeed, final double rotationSpeed) {
    differentialDrive.arcadeDrive(rotationSpeed, forwardSpeed);
  }

  public void autonomouseDrive() {
    leftMotor.set(pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.setPoint));
    rightMotor.set(pidController.calculate(rightEncoder.getDistance(), Constants.PidConstants.setPoint));
  }

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public Encoder getLeftEncoder(){
    return leftEncoder;
  }
  
  public Encoder getrightEncoder(){
    return rightEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
