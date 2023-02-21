// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveByTimeConstants;
import frc.robot.Constants.DriveByTimeConstants.*;


public class DriveByTime extends CommandBase {
  private final Chassis driveTrainSubsystem;
  private final double setTime;
  private double startTime;
  Timer timer=new Timer();
  /**
   * Creates a new ExampleCommand.
   
   * @param subsystem The subsystem used by this command.
   */
  public DriveByTime(final Chassis driveTrainSubsystem,double setTime) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this. setTime=setTime;
        
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime=timer.get();
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.drive(DriveByTimeConstants.DRIVE_BY_TIME_SPEED, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime=timer.get();
    return currentTime - startTime >=DriveByTimeConstants.SET_TIME_TO_GAME_PIECES ;
  }
}