// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.commands.CollectBySpeed;
import frc.robot.commands.DriveByDistance;
import frc.robot.commands.DriveBySpeed;
import frc.robot.commands.ShifterChangerCommand;
import frc.robot.commands.ShootBySpeed;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PneuomaticsSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis chassis = Chassis.getInstance();
  private final Collector collector = Collector.getInstance();
  private final Shooter shooter = Shooter.getInstantce();
  private final PneuomaticsSubsystem pneuomaticsSubsystem = PneuomaticsSubsystem.getInstance();

  SendableChooser<Command> chooser = new SendableChooser<>();

  public Joystick joystick = new Joystick(JoystickConstants.DRIVING_JOYSTICK_PORT);
  public XboxController xboxController = new XboxController(JoystickConstants.BUTTONS_JOYSTICK_PORT);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    chassis.setDefaultCommand(new DriveBySpeed(chassis, () -> joystick.getY(), () -> joystick.getZ()));

    /*chooser.addOption("short Straight", loadPathplannerTrajectory("C:\\Users\\Yony\\Desktop\\RoboticsProgramming\\AddingPID\\RoboTraining2023\\SemiWorkingPID2023\\src\\main\\deploy\\deploy\\pathplanner\\generatedJSON", true));
    chooser.addOption("long Straight", loadPathplannerTrajectory("/Users/Yony/Desktop/RoboticsProgramming/AddingPID/RoboTraining2023/SemiWorkingPID2023/src/main/deploy/deploy/pathplanner/generatedJSON/Long Straight Path.wpilib.json", true));
    chooser.addOption("curved path", loadPathplannerTrajectory("/Users/Yony/Desktop/RoboticsProgramming/AddingPID/RoboTraining2023/SemiWorkingPID2023/src/main/deploy/deploy/pathplanner/generatedJSON/Curved Path.wpilib.json", true));

    Shuffleboard.getTab("Autonomous").add(chooser);*/
  }

  public Command loadPathplannerTrajectory(String filename, boolean resetOdometry)
  {
    if (resetOdometry)
      chassis.resetOdometry(new Pose2d());

    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception) {
      DriverStation.reportError( exception.getMessage(), exception.getStackTrace());
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, chassis::getPose, new RamseteController(PidConstants.kRemeseteB, PidConstants.kRemeseteZeta), new SimpleMotorFeedforward(PidConstants.Ks, PidConstants.Kv, PidConstants.Ka), PidConstants.kDifferentialDriveKinematics, chassis::getWheelSpeeds, new PIDController(PidConstants.Kp, PidConstants.Ki, PidConstants.Kd), new PIDController(PidConstants.Kp, PidConstants.Ki, PidConstants.Kd), chassis::tankDriveWolts, chassis);
  
    if (resetOdometry) {
      return new SequentialCommandGroup(new InstantCommand(()->chassis.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    }
    else {
      return ramseteCommand;
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    //JoystickButton driveMeterOne = new JoystickButton(xboxController, XboxController.Button.kX.value);
    //driveMeterOne.onTrue(new DriveByDistance());
    JoystickButton collectButton = new JoystickButton(joystick, Constants.JoystickConstants.BUTTON_NUMBER_COLLECT);
    collectButton.whileTrue(new CollectBySpeed(collector));
    JoystickButton ChangeShifterState = new JoystickButton(joystick, 2);
    ChangeShifterState.onTrue(new ShifterChangerCommand(pneuomaticsSubsystem));

    JoystickButton shootButton = new JoystickButton(joystick, Constants.JoystickConstants.BUTTON_NUMBER_SHOOT);
    shootButton.whileTrue(new ShootBySpeed(shooter));

    JoystickButton Ramsete = new JoystickButton(xboxController, XboxController.Button.kX.value);
    Ramsete.onTrue(loadPathplannerTrajectory("paths/TwoMeterCoolPath.wpilib.json", true));
  }

  ///home/lvuser/deploy/src/main/deploy/pathplanner/generatedJSON/StraigthShort.wpilib.json
  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveByDistance();
  }
}
