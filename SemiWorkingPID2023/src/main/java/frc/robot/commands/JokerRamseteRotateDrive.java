// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class JokerRamseteRotateDrive extends CommandBase {
  private final Chassis chassis;
  private final double wantedAngle;

  private boolean finished;
  
  /** Creates a new JokerRamseteRotateDrive. */
  public JokerRamseteRotateDrive(Chassis chassis, double wantedAngle) {
    this.chassis = chassis;
    this.wantedAngle = wantedAngle;
    this.finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = chassis.ReachWantedAngle(wantedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
