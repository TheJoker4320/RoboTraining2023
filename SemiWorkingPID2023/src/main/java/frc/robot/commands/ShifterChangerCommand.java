// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneuomaticsSubsystem;

public class ShifterChangerCommand extends CommandBase {
  boolean originalState;
  boolean state;
  static int counter;
  PneuomaticsSubsystem pneuomaticsSubsystem;
  static{
    counter=0;
  }
  /** Creates a new ShifterChangerCommand. */
  public ShifterChangerCommand(PneuomaticsSubsystem pneuomaticsSubsystem) {

    this.pneuomaticsSubsystem = pneuomaticsSubsystem;
    addRequirements(pneuomaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    state = !pneuomaticsSubsystem.getState();
    counter++;
    SmartDashboard.putNumber("Executes", counter);
    pneuomaticsSubsystem.changeState(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
