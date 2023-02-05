// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneuomaticsConstants;

public class PneuomaticsSubsystem extends SubsystemBase {
  private Compressor compressor;
  DoubleSolenoid shifterDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneuomaticsConstants.FIRST_SOLENOID_PORT, PneuomaticsConstants.SECOND_SOLENOID_PORT);
  static PneuomaticsSubsystem instance = null;

  public static PneuomaticsSubsystem getInstance()
  {
    if (instance == null)
      instance = new PneuomaticsSubsystem();
    
    return instance;
  }
  
  /** Creates a new PneuomaticsSubsysten. */
  private PneuomaticsSubsystem() {
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    shifterDoublePCM.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Current State", getState());
    /*if (compressor.getPressure() > 60)
      compressor.disable();
    if (compressor.getPressure() < 30)
      compressor.enableDigital();*/
    // This method will be called once per scheduler run
  }

  public boolean getState()
  {
    if (shifterDoublePCM.get() == Value.kForward)
      return true;
    else
      return false;
  }
  public void changeState(boolean state) {
    if (state)
      shifterDoublePCM.set(Value.kForward);
    else
      shifterDoublePCM.set(Value.kReverse);
  }
}
