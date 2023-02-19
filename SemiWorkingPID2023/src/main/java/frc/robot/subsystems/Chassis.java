// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PidConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static double distantceTraveled;

  private final WPI_TalonSRX rightMotor;
  private final WPI_TalonSRX leftMotor;
  private final WPI_TalonSRX rightSlaveMotor;
  private final WPI_TalonSRX leftSlaveMotor;

  private final MotorControllerGroup rightMotorControllerGroup;
  private final MotorControllerGroup leftMotorControllerGroup;

  public static int counter;
  public final AHRS navXGyro = new AHRS();

  private Encoder rightEncoder;
  private Encoder leftEncoder;

  private final DifferentialDrive differentialDrive;
  private final DifferentialDriveOdometry differentialDriveOdometry;
  
  private static Chassis instance;
  private final PIDController pidController = new PIDController(Constants.PidConstants.Kp, Constants.PidConstants.Ki, Constants.PidConstants.Kd);
  public static Timer timer;
  public static double currentAccelTimeStamp;
  public static double prevAccelTimeStamp;
  public static double currentVelocityTimeStamp;
  public static double prevVelocityTimeStamp;
  public static double currentAccel;
  public static double prevAccel;
  public static double currentVelocity;
  public static double prevVelocity;

  public double overallDistance;

  public static Chassis getInstance() {
    if (instance == null) instance = new Chassis();
    return instance;
  }

  private Chassis() {

    finishedPartOne = false;
    finishedPartTwo = false;
    finishedPartThree = false;
    finishedPartFour = false;

    finishedSegments = new boolean[Constants.ConstantsForPath.RIGHT_SETPOINTS.length];
    for (int i = 0; i < finishedSegments.length; i++)
      finishedSegments[i] = false;
    //--------------------

    timer = new Timer();
    timer.start();
    overallDistance = 0;
    prevAccel = 0;
    prevAccelTimeStamp = 0;
    prevVelocity = 0;
    prevVelocityTimeStamp = 0;
    //previousState = false;

    //--------------------------------
    counter = 0;
    pidController.setTolerance(PidConstants.TOLERANCE);

    rightMotor = new WPI_TalonSRX(ChassisConstants.RIGHT_MASTER_MOTOR_PORT);
    rightMotor.configFactoryDefault();
    
    rightSlaveMotor = new WPI_TalonSRX(ChassisConstants.RIGHT_SLAVE_MOTOR_PORT);
    rightSlaveMotor.configFactoryDefault();
    rightSlaveMotor.follow(rightMotor);

    leftMotor = new WPI_TalonSRX(ChassisConstants.LEFT_MASTER_MOTOR_PORT);
    leftMotor.configFactoryDefault();

    leftSlaveMotor = new WPI_TalonSRX(ChassisConstants.LEFT_SLAVE_MOTOR_PORT);
    leftSlaveMotor.configFactoryDefault();
    leftSlaveMotor.follow(leftMotor);
    
    leftEncoder = new Encoder(ChassisConstants.LEFT_ENCODER_SCORE[0], ChassisConstants.LEFT_ENCODER_SCORE[1]);
    rightEncoder = new Encoder(ChassisConstants.RIGHT_ENCODER_SCORE[0], ChassisConstants.RIGHT_ENCODER_SCORE[1]);

    leftEncoder.setDistancePerPulse(Constants.PidConstants.RATIO_TICKS_TO_METERS);
    rightEncoder.setDistancePerPulse(Constants.PidConstants.RATIO_TICKS_TO_METERS);

    rightMotorControllerGroup = new MotorControllerGroup(rightMotor, rightSlaveMotor);
    leftMotorControllerGroup = new MotorControllerGroup(leftMotor, leftSlaveMotor);
    leftMotorControllerGroup.setInverted(true);
    
    differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

    navXGyro.resetDisplacement();
    navXGyro.reset();

    pidController.reset();
    rightEncoder.reset();
    leftEncoder.reset();

    differentialDriveOdometry = new DifferentialDriveOdometry(navXGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void drive(final double forwardSpeed, final double rotationSpeed) {
    /*Xacceleration = navXGyro.getWorldLinearAccelX();

    Xdisplacement = navXGyro.getDisplacementX();
    Ydisplacement = navXGyro.getDisplacementY();
    Zdisplacement = navXGyro.getDisplacementZ();
    overallDistance = Math.
    overallDistance = Math.sqrt(Xdisplacement * Xdisplacement + Ydisplacement * Ydisplacement + Zdisplacement * Zdisplacement);
    SmartDashboard.putNumber("NavX distantce", overallDistance);*/
    SmartDashboard.putNumber("Encoder Value Right", rightEncoder.getDistance() / rightEncoder.getDistancePerPulse());
    SmartDashboard.putNumber("Encoder Value Left", leftEncoder.getDistance() / leftEncoder.getDistancePerPulse());
    SmartDashboard.putNumber("Encoder Value Right - Meters",rightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value Left - Meters",leftEncoder.getDistance());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putNumber("Gyro yaw angle", navXGyro.getYaw());
    SmartDashboard.putNumber("Gyro Pitche angle", navXGyro.getPitch());
    SmartDashboard.putNumber("Gyro roll angle", navXGyro.getRoll());
 
    differentialDrive.arcadeDrive(-forwardSpeed, -rotationSpeed);
  }

  //-------------------------------------------------
  //-------------------------------------------------

  public boolean autonomouseDrive(double rightsetPoint, double leftSetpoint) {
    SmartDashboard.putNumber("Encoder Value Right - Meters",rightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value Left - Meters",leftEncoder.getDistance());
    SmartDashboard.putNumber("Gyro Heading", getHeading());

    double previousRightEncoderValue = rightEncoder.getDistance();
    double previousLeftEncoderValue = leftEncoder.getDistance();


    //differentialDrive.arcadeDrive(0,(pidController.calculate(leftEncoder.getDistance(), Constants.PidConstants.SET_POINT)));
    SmartDashboard.putNumber("Tolerance", pidController.getPositionTolerance());
    SmartDashboard.putNumber("PID Value Right", pidController.calculate(rightEncoder.getDistance(), rightsetPoint));
    SmartDashboard.putNumber("PID Value Left", pidController.calculate(leftEncoder.getDistance(), leftSetpoint));
    leftMotorControllerGroup.set(1 * (pidController.calculate(-1 * leftEncoder.getDistance(), leftSetpoint)));
    rightMotorControllerGroup.set(1 * (pidController.calculate(rightEncoder.getDistance(), rightsetPoint)));

    if (previousLeftEncoderValue == leftEncoder.getDistance() && previousRightEncoderValue == rightEncoder.getDistance() && previousLeftEncoderValue != 0 && previousRightEncoderValue != 0 )
      return true;
    else
      return false;
  }

  public boolean turnAngle(double wantedAngle)
  {
    double currentAngle = navXGyro.getYaw();
    currentAngle += currentAngle < 0 ? 360 : 0;
    
    double distatnceClockwise = wantedAngle - currentAngle;
    distatnceClockwise += distatnceClockwise < 0 ? 360 : 0;

    double distantceNotClockwise = 360 - distatnceClockwise;
    

    if (Math.min(distatnceClockwise, distantceNotClockwise) < 1 && Math.min(distatnceClockwise, distantceNotClockwise) > -1)
    {
      return true;
    }

    if (distatnceClockwise < distantceNotClockwise)
    {
      differentialDrive.arcadeDrive(0, distatnceClockwise/180);
      return false;
    }
    else
    {
      differentialDrive.arcadeDrive(0, -1 * distantceNotClockwise/180);
      return false;
    }

    
  }

  //-------------------------------------------------
  //-------------------------------------------------

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public Encoder getLeftEncoder(){
    return leftEncoder;
  }
  
  public Encoder getRightEncoder(){
    return rightEncoder;
  }

  //-------------------------------------------------------
  public AHRS getGyro() {
    return navXGyro;
  }
  public double getRightEncoderPosition() {
    return rightEncoder.getDistance();
  }
  public double getLeftEncoderPosition() {
    return leftEncoder.getDistance();
  }
  public double getRightEncoderVelocity() {
    return rightEncoder.getRate();
  }
  public double getLeftEncoderVelocity() {
    return leftEncoder.getRate();
  }
  public double getAverageEncoderDistantce() {
    return (getLeftEncoderPosition() + getRightEncoderPosition())/2;
  }
  public double getTurnRate() {
    return -navXGyro.getRate();
  }
  public double getHeading() {
    return navXGyro.getRotation2d().getDegrees();
  }
  public Pose2d getPose() {
    return differentialDriveOdometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    differentialDriveOdometry.resetPosition(navXGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    counter++;
    SmartDashboard.putNumber("This was called - times", counter);
    rightMotorControllerGroup.setVoltage(rightVolts*10/12);
    leftMotorControllerGroup.setVoltage(leftVolts*10/12);
    /*rightMotor.configVoltageCompSaturation(rightVolts);
    leftMotor.configVoltageCompSaturation(leftVolts);
    rightMotor.enableVoltageCompensation(true);
    leftMotor.enableVoltageCompensation(true);*/
    differentialDrive.feed();
  }
  //-------------------------------------------------------
  public void stopDriving() {
    rightMotorControllerGroup.set(0);
    leftMotorControllerGroup.set(0);
  }

  public void autonomousDrive()
  {
    rightMotorControllerGroup.set(pidController.calculate(rightEncoder.getDistance(), PidConstants.SET_POINT));
    leftMotorControllerGroup.set(pidController.calculate(-1 * leftEncoder.getDistance(), PidConstants.SET_POINT));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(navXGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    // This method will be called once per scheduler run

    currentAccelTimeStamp = timer.get();
    currentAccel = navXGyro.getWorldLinearAccelX();

    currentVelocity = (currentAccelTimeStamp - prevAccelTimeStamp) * ((currentAccel + prevAccel)/2);

    currentVelocityTimeStamp = (currentAccelTimeStamp + prevAccelTimeStamp)/2;
    overallDistance += (currentVelocityTimeStamp - prevVelocityTimeStamp) * ((currentVelocity + prevVelocity)/2);

    prevAccel = currentAccel;
    prevAccelTimeStamp = currentAccelTimeStamp;
    prevVelocity = currentVelocity;
    prevVelocityTimeStamp = currentVelocityTimeStamp;

    
  }

  //-----------------------------------------------------------------
  //-----------RAMSETE COMMAND REPLACEMENT WITH MANUAL PID-----------
  //-----------------------------------------------------------------

  public boolean finishedPathSegment(int segment, double tolerance)
  {
    if (segment == 1)
    {
      boolean rightCorrect = (Constants.ConstantsForPath.RIGHT_SETPOINT_ONE + tolerance > rightEncoder.getDistance() && Constants.ConstantsForPath.RIGHT_SETPOINT_ONE - tolerance < rightEncoder.getDistance());
      boolean leftCorrect = (Constants.ConstantsForPath.LEFT_SETPOINT_ONE + tolerance > leftEncoder.getDistance() && Constants.ConstantsForPath.LEFT_SETPOINT_ONE - tolerance < leftEncoder.getDistance());

      return (rightCorrect && leftCorrect);
    }
    else if (segment == 2)
    {
      boolean rightCorrect = (Constants.ConstantsForPath.RIGHT_SETPOINT_TWO + tolerance > rightEncoder.getDistance() && Constants.ConstantsForPath.RIGHT_SETPOINT_TWO - tolerance < rightEncoder.getDistance());
      boolean leftCorrect = (Constants.ConstantsForPath.LEFT_SETPOINT_TWO + tolerance > leftEncoder.getDistance() && Constants.ConstantsForPath.LEFT_SETPOINT_TWO - tolerance < leftEncoder.getDistance());

      return (rightCorrect && leftCorrect);
    }
    else if (segment == 3)
    {
      boolean rightCorrect = (Constants.ConstantsForPath.RIGHT_SETPOINT_THREE + tolerance > rightEncoder.getDistance() && Constants.ConstantsForPath.RIGHT_SETPOINT_THREE - tolerance < rightEncoder.getDistance());
      boolean leftCorrect = (Constants.ConstantsForPath.LEFT_SETPOINT_THREE + tolerance > leftEncoder.getDistance() && Constants.ConstantsForPath.LEFT_SETPOINT_THREE - tolerance < leftEncoder.getDistance());

      return (rightCorrect && leftCorrect);
    }
    else if (segment == 4)
    {
      boolean rightCorrect = (Constants.ConstantsForPath.RIGHT_SETPOINT_FOUR + tolerance > rightEncoder.getDistance() && Constants.ConstantsForPath.RIGHT_SETPOINT_FOUR - tolerance < rightEncoder.getDistance());
      boolean leftCorrect = (Constants.ConstantsForPath.LEFT_SETPOINT_FOUR + tolerance > leftEncoder.getDistance() && Constants.ConstantsForPath.LEFT_SETPOINT_FOUR - tolerance < leftEncoder.getDistance());

      return (rightCorrect && leftCorrect);
    }
    else
      return false;
  }

  boolean finishedPartOne;
  boolean finishedPartTwo;
  boolean finishedPartThree;
  boolean finishedPartFour;

  //----------------------------------

  boolean[] finishedSegments;

  public boolean followPathMoreSegments()
  {
    SmartDashboard.putNumber("Encoder Value Right - Meters",rightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value Left - Meters",leftEncoder.getDistance());

    for (int i = 0; i < finishedSegments.length; i++)
    {
      if (finishedSegments[i] = false)
      {
        leftMotorControllerGroup.set(1 * pidController.calculate(-1 * leftEncoder.getDistance(), Constants.ConstantsForPath.LEFT_SETPOINTS[i]));
        rightMotorControllerGroup.set(1 * pidController.calculate(1 * rightEncoder.getDistance(), Constants.ConstantsForPath.RIGHT_SETPOINTS[i]));

        if (i == finishedSegments.length - 1)
          finishedSegments[i] = finishedSegmentsMoreSegments(i, 0.01);
        else
          finishedSegments[i] = finishedSegmentsMoreSegments(i, 0.03);
        break;
      }
    }

    return finishedSegments[finishedSegments.length - 1];
  }

  public boolean finishedSegmentsMoreSegments(int segmentIndex, double tolerance)
  {
    boolean rightCorrect = (rightEncoder.getDistance() > Constants.ConstantsForPath.RIGHT_SETPOINTS[segmentIndex] - tolerance && rightEncoder.getDistance() < Constants.ConstantsForPath.RIGHT_SETPOINTS[segmentIndex] + tolerance);
    boolean leftCorrect = (leftEncoder.getDistance() > Constants.ConstantsForPath.LEFT_SETPOINTS[segmentIndex] - tolerance && leftEncoder.getDistance() < Constants.ConstantsForPath.LEFT_SETPOINTS[segmentIndex] + tolerance);

    return (rightCorrect && leftCorrect);
  }

  //----------------------------------

  public boolean followPath()
  {
    SmartDashboard.putNumber("Encoder Value Right - Meters",rightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value Left - Meters",leftEncoder.getDistance());

    if (!finishedPartOne)
    {
      leftMotorControllerGroup.set(1 * (pidController.calculate(-1 * leftEncoder.getDistance(), Constants.ConstantsForPath.LEFT_SETPOINT_ONE)));
      rightMotorControllerGroup.set(1 * (pidController.calculate(rightEncoder.getDistance(), Constants.ConstantsForPath.RIGHT_SETPOINT_ONE)));

      finishedPartOne = finishedPathSegment(1, 0.03);
    }
    else if (!finishedPartTwo)
    {
      leftMotorControllerGroup.set(1 * (pidController.calculate(-1 * leftEncoder.getDistance(), Constants.ConstantsForPath.LEFT_SETPOINT_TWO)));
      rightMotorControllerGroup.set(1 * (pidController.calculate(rightEncoder.getDistance(), Constants.ConstantsForPath.RIGHT_SETPOINT_TWO)));

      finishedPartTwo = finishedPathSegment(2, 0.03);
    }
    else if (!finishedPartThree)
    {
      leftMotorControllerGroup.set(1 * (pidController.calculate(-1 * leftEncoder.getDistance(), Constants.ConstantsForPath.LEFT_SETPOINT_THREE)));
      rightMotorControllerGroup.set(1 * (pidController.calculate(rightEncoder.getDistance(), Constants.ConstantsForPath.RIGHT_SETPOINT_THREE)));

      finishedPartThree = finishedPathSegment(3, 0.03);
    }
    else if (!finishedPartFour)
    {
      leftMotorControllerGroup.set(1 * (pidController.calculate(-1 * leftEncoder.getDistance(), Constants.ConstantsForPath.LEFT_SETPOINT_FOUR)));
      rightMotorControllerGroup.set(1 * (pidController.calculate(rightEncoder.getDistance(), Constants.ConstantsForPath.RIGHT_SETPOINT_FOUR)));

      finishedPartFour = finishedPathSegment(4, 0.01);
    }

    return finishedPartFour;
  }

  /*
   * THE JOKER RAMSETTE COMMAND:
   * 
   * - Functions for turning the robot in the z axis at the current location
   * - Function for doing pid calculations based on measurment and input
   * - Functions the gives the motors ouput
   */

  public boolean ReachWantedAngle(double wantedAngle)
  {
    double currentAngle = navXGyro.getYaw() < 0 ? navXGyro.getYaw() + 360 : navXGyro.getYaw();

    double distantceClockwise = wantedAngle - currentAngle;
    distantceClockwise += distantceClockwise < 0 ? 360 : 0;
    double distantceAgainstClockwise = 360 - distantceClockwise;

    if (Math.min(distantceClockwise, distantceAgainstClockwise) < 1)
    {
      return true;
    }

    if (distantceClockwise < distantceAgainstClockwise)
    {
      double turningSpeed = pidController.calculate(distantceClockwise, 0);
      if (turningSpeed < 0.1)
        turningSpeed = 0.1;

      rightMotorControllerGroup.set(-turningSpeed);
      leftMotorControllerGroup.set(turningSpeed);
      return false;
    }
    else
    {
      double turningSpeed = pidController.calculate(distantceAgainstClockwise, 0);
      if (turningSpeed < 0.1)
        turningSpeed = 0.1;

      rightMotorControllerGroup.set(turningSpeed);
      leftMotorControllerGroup.set(-turningSpeed);
      return false;
    }
  }

  //----------

  public double pidCalculate(double measurement, double setpoint, int type) {
    return pidController.calculate(measurement, setpoint);
  }

  //----------

  public void setPowerToMotors(double rightValue, double leftValue) {
    rightMotorControllerGroup.set(rightValue);
    leftMotorControllerGroup.set(leftValue);
  }
}