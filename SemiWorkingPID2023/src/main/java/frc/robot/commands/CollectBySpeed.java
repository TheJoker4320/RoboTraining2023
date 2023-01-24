// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectBySpeed extends CommandBase {
  private final Collector collectorSubSystem;
  
  /** Creates a new CollectBySpeed. */
  public CollectBySpeed(Collector collectorSubSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collectorSubSystem = collectorSubSystem;

    addRequirements(collectorSubSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collectorSubSystem.collect();
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
