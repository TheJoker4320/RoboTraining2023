// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class JokerRamseteFollowCommands extends CommandBase {
  private final CommandBase[] commands;
  private boolean[] finishedCommands;

  private int currentCommand;

  /** Creates a new DriveCommandsPath. */
  public JokerRamseteFollowCommands(CommandBase[] commands, Chassis chassis) {
    this.commands = commands;

    finishedCommands = new boolean[commands.length];
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < finishedCommands.length; i++)
      finishedCommands[i] = false;

    currentCommand = 0;
    commands[0].initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commands[currentCommand].execute();
    finishedCommands[currentCommand] = commands[currentCommand].isFinished();

    if (finishedCommands[currentCommand]) {
      commands[currentCommand].end(false);
      currentCommand++;
      commands[currentCommand].initialize();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commands[commands.length - 1].isFinished();
  }
}
