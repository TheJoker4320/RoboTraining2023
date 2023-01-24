// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final TalonSRX shooterMotor;
  private static Shooter instantce = null;

  public static Shooter getInstantce() {
    if (instantce == null) {
      instantce = new Shooter();
      return instantce;
    }
    else
      return instantce;
  }

  /** Creates a new Shooter. */
  private Shooter() {
    shooterMotor = new TalonSRX(Constants.ShooterConstants.SHOOTER_MOTOR_PORT);
    shooterMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    shooterMotor.set(TalonSRXControlMode.PercentOutput, 1);
  }
}
