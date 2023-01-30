// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveBySpeed extends CommandBase {
  private final Chassis driveTrainSubsystem;
  private final DoubleSupplier forwardSpeed;
  private final DoubleSupplier rotationSpeed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveBySpeed(final Chassis driveTrainSubsystem, final DoubleSupplier forwardSpeed, final DoubleSupplier rotationSpeed) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.forwardSpeed = forwardSpeed;
    this.rotationSpeed = rotationSpeed;
        
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.drive(forwardSpeed.getAsDouble(), rotationSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}