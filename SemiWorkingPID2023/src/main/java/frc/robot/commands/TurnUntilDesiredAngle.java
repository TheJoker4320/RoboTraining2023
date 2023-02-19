// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class TurnUntilDesiredAngle extends CommandBase {
  /** Creates a new TurnUntilDesiredAngle. */

  private Chassis chassis;
  private double requestedAngle;
  private boolean reachedAngle;

  public TurnUntilDesiredAngle(Chassis chassis, double requestedAngle) {
    this.chassis = chassis;
    this.requestedAngle = requestedAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.navXGyro.reset();
    reachedAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //reachedAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.navXGyro.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
