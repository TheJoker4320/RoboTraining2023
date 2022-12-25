// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.ChassisConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_VictorSPX rightMotor;
  private final WPI_VictorSPX leftMotor;
  private final WPI_VictorSPX rightSlaveMotor;
  private final WPI_VictorSPX leftSlaveMotor;

  private final DifferentialDrive differentialDrive;
  private static Chassis instance;

  public static Chassis getInstance() {
    if (instance == null) instance = new Chassis();
    return instance;
  }

  private Chassis() {
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
    
    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
  }

  public void drive(final double forwardSpeed, final double rotationSpeed) {
    differentialDrive.arcadeDrive(rotationSpeed, forwardSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
