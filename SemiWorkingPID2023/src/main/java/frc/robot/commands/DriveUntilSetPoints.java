// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveUntilSetPoints extends CommandBase {
  private double rightSetpoint;
  private double leftSetpoint;
  private Chassis chassis;
  private boolean finished;

  /** Creates a new DriveUntilSetPoints. */
  public DriveUntilSetPoints(Chassis chassis, double rightSetpoint, double leftSetpoint) {
    this.chassis = chassis;
    this.rightSetpoint = rightSetpoint;
    this.leftSetpoint = leftSetpoint;

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = chassis.autonomouseDrive(rightSetpoint, leftSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
