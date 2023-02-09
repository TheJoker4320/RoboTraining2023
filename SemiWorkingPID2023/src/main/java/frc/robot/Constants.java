// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ChassisConstants {
        public static final int RIGHT_MASTER_MOTOR_PORT = 2;
        public static final int RIGHT_SLAVE_MOTOR_PORT = 7;
        public static final int LEFT_MASTER_MOTOR_PORT = 1;
        public static final int LEFT_SLAVE_MOTOR_PORT = 6;
        
        public static final boolean SET_INVERTED = true;
        public static final int[] LEFT_ENCODER_SCORE= {2,3};
        public static final int[] RIGHT_ENCODER_SCORE= {0,1};

    }
    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_PORT = 12;
    }

    public static final class JoystickConstants {
        public static final int DRIVING_JOYSTICK_PORT = 1;
        public static final int BUTTONS_JOYSTICK_PORT = 0;

        public static final int BUTTON_NUMBER_COLLECT = 1;
        public static final int BUTTON_NUMBER_SHOOT = 5;
    }

    public static final class CollectorConstants {
        public static final int RIGHT_COLLECTOR_MOTOR = 8;
        public static final int LEFT_COLLECTOR_MOTOR = 9;
    }

    public static final class PidConstants {
        //SYSID - KP = 1.5594, 1.0051
        public static final double Kp = 1;
        public static final double Ki = 0;
        public static final double Kd = 0.4;
        //------------------------------------
        public static final double Ks = 0.75;
        public static final double Kv = 1;
        public static final double Ka = 1;

        public static final double kTrackWidth = 0.7;
        public static final DifferentialDriveKinematics kDifferentialDriveKinematics= new DifferentialDriveKinematics(kTrackWidth);
        public static final double kMaxSpeed = 3.0;
        public static final double kMaxAcceleration = 3.0;
        public static final double kRemeseteB = 2;
        public static final double kRemeseteZeta = 0.7;

        public static final double TOLERANCE = 0.01; //6cm
        public static final double SET_POINT = 2;
        // TODO: Add calculation explaining why the gear ratio is 5.0:1
        public static final double GEAR_RATIO_ENCODER = (double)27/5;
        public static final int TICKS_IN_ENCODER = 256;  
        public static final double CIRCUMFARANCE = Math.PI * 0.0970; //DO NOT CHANGE!!!!
        public static final double ONE_WHEEL_TO_TICKS = GEAR_RATIO_ENCODER * TICKS_IN_ENCODER;

        public static final double RATIO_TICKS_TO_METERS = CIRCUMFARANCE / ONE_WHEEL_TO_TICKS;
    }

    public static final class PneuomaticsConstants {
        public static final int FIRST_SOLENOID_PORT = 1;
        public static final int SECOND_SOLENOID_PORT = 0;
    }
}
