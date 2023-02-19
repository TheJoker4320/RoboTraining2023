// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PidConstants;
import frc.robot.subsystems.Chassis;

public class JokerRamsteTranslateDrive extends CommandBase {
  /** Creates a new JokerRamsteTranslateDrive. */
  private final Chassis chassis;
  private final double rightSetpoint;
  private final double leftSetpoint;

  public JokerRamsteTranslateDrive(Chassis chassis, double rightSetpoint, double leftSetpoint) {
    this.chassis = chassis;
    this.rightSetpoint = rightSetpoint;
    this.leftSetpoint = leftSetpoint;
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightValue = chassis.pidCalculate(chassis.getRightEncoder().getDistance(), rightSetpoint, 1);
    double leftValue = chassis.pidCalculate(chassis.getLeftEncoder().getDistance(), leftSetpoint, 1);

    chassis.setPowerToMotors(rightValue, leftValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rightEncoderCorrect = (chassis.getRightEncoder().getDistance() > rightSetpoint - PidConstants.TOLERANCE && chassis.getRightEncoder().getDistance() < rightSetpoint + PidConstants.TOLERANCE);
    boolean leftEncoderCorrect = (chassis.getLeftEncoder().getDistance() > leftSetpoint - PidConstants.TOLERANCE && chassis.getLeftEncoder().getDistance() < leftSetpoint + PidConstants.TOLERANCE);

    return rightEncoderCorrect && leftEncoderCorrect;
  }
}
