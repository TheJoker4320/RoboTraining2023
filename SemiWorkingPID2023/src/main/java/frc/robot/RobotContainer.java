// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.CollectBySpeed;
import frc.robot.commands.DriveByDistance;
import frc.robot.commands.DriveBySpeed;
import frc.robot.commands.ShootBySpeed;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
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

  public Joystick joystick = new Joystick(JoystickConstants.DRIVING_JOYSTICK_PORT);
  public XboxController xboxController = new XboxController(JoystickConstants.BUTTONS_JOYSTICK_PORT);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    /*chassis.setDefaultCommand(new DriveBySpeed(chassis, () -> joystick.getY(), () -> joystick.getZ() * -1));*/

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    JoystickButton driveMeterOne = new JoystickButton(xboxController, XboxController.Button.kX.value);
    driveMeterOne.onTrue(new DriveByDistance());
    JoystickButton collectButton = new JoystickButton(joystick, Constants.JoystickConstants.BUTTON_NUMBER_COLLECT);
    collectButton.whileTrue(new CollectBySpeed(collector));
    
    JoystickButton shootButton = new JoystickButton(joystick, Constants.JoystickConstants.BUTTON_NUMBER_SHOOT);
    shootButton.whileTrue(new ShootBySpeed(shooter));
    chassis.setDefaultCommand(new DriveBySpeed(chassis, () -> joystick.getY(), () -> joystick.getZ()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
