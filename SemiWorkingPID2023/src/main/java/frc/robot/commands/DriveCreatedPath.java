// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveCreatedPath extends CommandBase {
  public Chassis chassis;
  public boolean finishedPath;

  /** Creates a new DriveCreatedPath. */
  public DriveCreatedPath(Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
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
    finishedPath = chassis.followPath();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedPath;
  }
}