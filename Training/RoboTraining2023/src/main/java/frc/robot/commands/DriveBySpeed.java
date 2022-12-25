package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import java.util.function.DoubleSupplier;

public class DriveBySpeed extends CommandBase {

  private final DoubleSupplier forwardSpeedSupplier;
  private final DoubleSupplier rotationSpeedSupplier;
  private final Chassis chassis;

  /**
   * Activates the chassis's talons with the given parameters.
   * @param chassis Chassis object.
   * @param forwardSpeedSupplier The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotationSpeedSupplier robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
   */
  public DriveBySpeed(final Chassis chassis, final DoubleSupplier forwardSpeedSupplier, final DoubleSupplier rotationSpeedSupplier) {
    this.chassis = chassis;
    this.forwardSpeedSupplier = forwardSpeedSupplier;
    this.rotationSpeedSupplier = rotationSpeedSupplier;
    addRequirements(this.chassis); // Added to avoid errors when more than one subsystem is using the command.
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    chassis.arcadeDrive(forwardSpeedSupplier.getAsDouble(), rotationSpeedSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(final boolean interrupted) {
    chassis.stop();
  }
}