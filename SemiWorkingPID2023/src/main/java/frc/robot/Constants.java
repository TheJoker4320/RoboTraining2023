// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final int RIGHT_SLAVE_MOTOR_PORT = 3;
        public static final int LEFT_MASTER_MOTOR_PORT = 14;
        public static final int LEFT_SLAVE_MOTOR_PORT = 16;
        
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
        public static final double Kp = 1.40745825;
        public static final double Ki = 0;
        public static final double Kd = 0.1;

        public static final double TOLERANCE = 0.06; //6cm
        public static final double SET_POINT = 1;
        // TODO: Add calculation explaining why the gear ratio is 5.0:1
        public static final double GEAR_RATIO = 5.25;
        public static final int TICKS_IN_ENCODER = 256;  
        public static final double CIRCUMFARANCE = Math.PI * 0.1016;
        public static final double ONE_WHEEL_TO_TICKS = GEAR_RATIO * TICKS_IN_ENCODER;

        public static final double RATIO_TICKS_TO_METERS = CIRCUMFARANCE / ONE_WHEEL_TO_TICKS;
    }

    public static final class PneuomaticsConstants {
        public static final int FIRST_SOLENOID_PORT = 0;
        public static final int SECOND_SOLENOID_PORT = 1;
    }
}
