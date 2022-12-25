package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.ChassisConstants.*;

public class Chassis extends SubsystemBase {
  
  private final WPI_TalonSRX rightMaster;
  private final WPI_TalonSRX leftMaster;
  private final DifferentialDrive differentialDrive;
  private static Chassis single_instance = null;
  
  /**
   * Creates a new Chassis subsystem.
   */
  private Chassis() {
    // Tempoary constructors since VSCode is making problems
    rightMaster = new WPI_TalonSRX(RIGHT_MASTER_MOTOR_PORT);
    rightMaster.configFactoryDefault();
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftMaster = new WPI_TalonSRX(LEFT_MASTER_MOTOR_PORT);
    leftMaster.configFactoryDefault();
    leftMaster.setNeutralMode(NeutralMode.Brake);
    
    differentialDrive = null; //new DifferentialDrive(leftMaster, rightMaster);
    differentialDrive.setSafetyEnabled(false);
  }

  /**
   * Returns a Chassis subsystem.
   */
  public static Chassis getInstance()
    {
        if (single_instance == null)
            single_instance = new Chassis();
 
        return single_instance;
    }

  /**
   * Moves the talons according to the given paremeters.
   * @param forwardSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotationSpeedThe robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
   */
  public void arcadeDrive(final double forwardSpeed, final double rotationSpeed) {
    differentialDrive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  /**
   * When called, stops the chassis's talons.
   */
  public void stop(){
    differentialDrive.stopMotor();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // asaf was here lol
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
