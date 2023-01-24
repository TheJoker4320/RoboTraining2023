// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {

  private final TalonSRX rightCollectorMotor;
  private final TalonSRX leftCollectorMotor;

  private static Collector instatnce = null;

  public static Collector getInstance()
  {
    if (instatnce == null)
    {
      instatnce = new Collector();
      return instatnce;
    }
    else
      return instatnce;
  }

  /** Creates a new Collector. */
  private Collector() {
    rightCollectorMotor = new TalonSRX(Constants.CollectorConstants.RIGHT_COLLECTOR_MOTOR);
    rightCollectorMotor.configFactoryDefault();

    leftCollectorMotor = new TalonSRX(Constants.CollectorConstants.LEFT_COLLECTOR_MOTOR);
    leftCollectorMotor.configFactoryDefault();

    leftCollectorMotor.follow(rightCollectorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void collect()
  {
    rightCollectorMotor.set(TalonSRXControlMode.PercentOutput, 1);
  }
}
