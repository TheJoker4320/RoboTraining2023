// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PidConstants;
import frc.robot.subsystems.Chassis;

public class DriveCreatedPath extends CommandBase {
  public Chassis chassis;
  public boolean finishedPath;
  private OnyxRamseteCommand command;
  private Trajectory trajectory;

  /** Creates a new DriveCreatedPath. */
  public DriveCreatedPath(Chassis chassis, Path path) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    try {
      this.trajectory = TrajectoryUtil.fromPathweaverJson(path);
    } catch(IOException exception) {
      DriverStation.reportError( exception.getMessage(), exception.getStackTrace());
      this.trajectory = null;
    }
    
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command = new OnyxRamseteCommand(
                trajectory,
                chassis::getPose,
                new RamseteController(),
                PidConstants.feedforward,
                PidConstants.feedforward,
                PidConstants.kDifferentialDriveKinematics,
                chassis::getWheelSpeeds,
                new PIDController(PidConstants.Kp, 0, 0),
                new PIDController(PidConstants.Kp, 0, 0),
                chassis::tankDriveVolts,
                chassis
        );
        command.initialize();
  }

  @Override
    public void execute() {
        SmartDashboard.getNumber("Gyro Heading", chassis.navXGyro.getRotation2d().getDegrees());
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        chassis.stopDriving();
    }
}
