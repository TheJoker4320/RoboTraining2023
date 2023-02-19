// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class DriveByDistance extends CommandBase {

  public boolean firstMeterFinished;
  public boolean justFinishedDrive;

  public boolean turn90DegreesFinished;
  public boolean finishedSecondMeter;

  public boolean justFinishedTurning;
  private final Chassis chassis = Chassis.getInstance();
  /** Creates a new DriveByDistance. */
  public DriveByDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*firstMeterFinished = false;
    turn90DegreesFinished = false;
    finishedSecondMeter = false;

    chassis.navXGyro.reset();
    chassis.resetEncoders();*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (!firstMeterFinished)
    {
      //firstMeterFinished = chassis.autonomouseDrive(1);
      justFinishedDrive = firstMeterFinished;
    }
    else if (!turn90DegreesFinished)
    {
      if (justFinishedDrive)
      {
        justFinishedDrive = false;
      }

      turn90DegreesFinished = chassis.turnAngle(chassis.navXGyro.getYaw() + 90);
      justFinishedTurning = turn90DegreesFinished;
    }
    else if (!finishedSecondMeter) {
      if (justFinishedTurning)
      {
        justFinishedTurning = false;
        chassis.resetEncoders();
      }
      
      //finishedSecondMeter = chassis.autonomouseDrive(1);
    }*/
    chassis.autonomousDrive();

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
