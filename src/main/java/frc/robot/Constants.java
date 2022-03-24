// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // PNEUMATIC CONTROL MODULE
    public static final int INTAKE_SOLENOID = 0;
    public static final int ARM_SWING_SOLENOID = 1;

    // CAN ID
    public static final int RIGHT_DRIVE_MOTOR_FRONT = 1;
    public static final int RIGHT_DRIVE_MOTOR_BACK = 2;
    public static final int LEFT_DRIVE_MOTOR_FRONT = 3;
    public static final int LEFT_DRIVE_MOTOR_BACK = 4;
    public static final int RIGHT_ARM_MOTOR = 5;
    public static final int LEFT_ARM_MOTOR = 6;
    public static final int INTAKE_MOTOR = 7;
    public static final int STORAGE_MOTOR = 8;
    public static final int FEED_MOTOR = 9;
    public static final int RIGHT_SHOOTER_MOTOR = 10;
    public static final int LEFT_SHOOTER_MOTOR = 11;
    public static final int ELEVATION_ANGLE_MOTOR = 12;
    public static final int AZIMUTH_MOTOR = 13;

    // MOTOR SPEEDS
    public static final double LIFT_ARM_SPEED = 0;
    public static final double INTAKE_SPEED = 0;
    public static final double DRIVE_MAX_SPEED = 0.5;

    // DRIVER CONTROLS
    public static final int DRIVER_ONE = 0;
    public static final int PS4_LEFT_STICK_Y_AXIS = 1;
    public static final int PS4_LEFT_STICK_X_AXIS = 0;
    public static final int PS4_RIGHT_STICK_Y_AXIS = 5;
    public static final int PS4_RIGHT_STICK_X_AXIS = 2;

    // DriveTrain Navigation Constants
    public static final double ENCODER_ROTATION_UNITS = 4096;
    public static final double DRIVETRAIN_GEAR_RATIO = 0.1647058823;
    public static final double DRIVETRAIN_ROTATION_DISTANCE_RATIO = Math.PI * 0.666666666;
    public static final double MAX_WAYPOINT_ERROR = 0.5;
    public static final double MAX_DRIVE_HEADING_ERROR = 10;
    public static final double DRIVETRAIN_CURRENT_LIMIT = 40;
    public static final double DRIVETRAIN_POWER_RAMP_TIME = 0.1;

    // Drivetrain Controller Constants
    public static final double DRIVETRAIN_HEADING_PROPORTIONAL_GAIN = 0.1;
    public static final double DRIVETRAIN_SPEED_PROPORTIONAL_GAIN = 0.5;
    public static final double DRIVETRAIN_HEADING_INTEGRAL_GAIN = 0;
    public static final double DRIVETRAIN_SPEED_INTEGRAL_GAIN = 0;
    public static final double DRIVETRAIN_HEADING_DERIVITIVE_GAIN = 0;
    public static final double DRIVETRAIN_SPEED_DERIVITIVE_GAIN = 0;
    public static final double DRIVETRAIN_HEADING_MAX_PROPORTIONAL = 1;
    public static final double DRIVETRAIN_SPEED_MAX_PROPORTIONAL = 1;
    public static final double DRIVETRAIN_HEADING_MAX_INTEGRAL = 0;
    public static final double DRIVETRAIN_SPEED_MAX_INTEGRAL = 0;
    public static final double DRIVETRAIN_HEADING_MAX_DERIVITIVE = 0;
    public static final double DRIVETRAIN_SPEED_MAX_DERIVITIVE = 0;
    public static final double DRIVETRAIN_HEADING_MAX_POWER = 0.5;
    public static final double DRIVETRAIN_SPEED_MAX_POWER = 0.5;
    public static final double DRIVETRAIN_HEADING_SPEED = 0.5;
    public static final double DRIVETRAIN_SPEED_SPEED = 0.5;
    
    // ARM CONSTANTS
    public static final double MAX_ARM_ERROR = 40000;
    public static final double ARM_SLOW_SPEED = 1;
    public static final double ARM_FAST_SPEED = 3;
    public static final double FIRST_HOOK_POSITION = 150000;
    public static final double MIN_ARM_POSITION = 0;
    public static final double ALMOST_MIN_POSITION = 50000;
    public static final double HALF_ARM_POSITION = 100000;
    public static final double MAX_ARM_POSITION = 200000;
    public static final double ACCEPTABLE_ERROR = 8000;
    public static final double ARM_CURRENT_LIMIT = 40;
    public static final double ARM_POWER_RAMP_TIME = 0.5;

    // Arm Controller Constants
    public static final double ARM_PROPORTIONAL_GAIN = 0.00001;
    public static final double ARM_INTEGRAL_GAIN = 0.00000003;
    public static final double ARM_DERIVITIVE_GAIN = 0;
    public static final double MAX_ARM_PROPORTIONAL = 1;
    public static final double MAX_ARM_INTEGRAL = 0.25;
    public static final double MAX_ARM_DERIVITIVE = 0;
    public static final double ARM_MAX_POWER = 1;

    // Storage Constants
    public static final double FEED_POWER = 0.7;
    public static final double STORAGE_POWER = 0.5;

    // Shooter Constants
    public static final int SLIDER_AXIS = 3;
    public static final double SHOOTER_POWER_OFFSET = 0.1;
    public static final double SHOOTER_CURRENT_LIMIT = 20;
    public static final double SHOOTER_POWER_RAMP_TIME = 0.5;

    // Shooter Controller Constants
    public static final double SHOOTER_PORPORTIONAL_GAIN = 0.000006;
    public static final double SHOOTER_INTERGRAL_GAIN = 0.000003;
    public static final double SHOOTER_DERIVITIVE_GAIN = 0;
    public static final double SHOOTER_MAX_PROPORTIONAL = 1;
    public static final double SHOOTER_MAX_INTEGRAL = 1;
    public static final double SHOOTER_MAX_DERIVITIVE = 0;

    // Elevation Angle Constants
    public static final double ELEVATION_ANGLE_GEAR_RATIO = 0.145;
    public static final double SHOOTER_ELEVATION_ANGLE_UPPER_LIMIT = 80; //Actually 86
    public static final double SHOOTER_ELEVATION_ANGLE_LOWER_LIMIT = 30;
    public static final double ELEVATION_SPEED = 0.1;

    // Elevation Angle Controller Constants
    public static final double ELEVATION_ANGLE_PROPORTIONAL_GAIN = 0.5;
    public static final double ELEVATION_ANGLE_INTEGRAL_GAIN = 0;
    public static final double ELEVATION_ANGLE_DERIVITIVE_GAIN = 0;
    public static final double ELEVATION_ANGLE_MAX_PROPORTIONAL = 1;
    public static final double ELEVATION_ANGLE_MAX_INTEGRAL = 0;
    public static final double ELEVATION_ANGLE_MAX_DERIVITIVE = 0;

    // Azimuth Constants
    public static final double AZIMUTH_LOWER_LIMIT = -90;
    public static final double AZIMUTH_UPPER_LIMIT = 90;
    public static final double AZIMUTH_CURRENT_LIMIT = 40;
    public static final double AZIMUTH_POWER_RAMP_TIME = 0.5;
    public static final double AZIMUTH_GEAR_RATIO = 0.140;
    public static final double AZIMUTH_MAX_ERROR = 4;

    // Azimuth Controller Constants
    public static final double AZIMUTH_PROPORTIONAL_GAIN = 0.009;
    public static final double AZIMUTH_INTEGRAL_GAIN = 0.0007;
    public static final double AZIMUTH_DERIVITIVE_GAIN = 0.03;
    public static final double AZIMUTH_MAX_PROPORTIONAL = 1;
    public static final double AZIMUTH_MAX_INTEGRAL = 0.1;
    public static final double AZIMUTH_MAX_DERIVITIVE = 0.75;
    public static final double AZIMUTH_MAX_POWER = 1;
    public static final double AZIMUTH_SPEED = 1;

    // Color Constants
    public static final Color RED_ALLIANCE_COLOR = new Color(0.6,0.3,0.1);
    public static final Color BLUE_ALLIANCE_COLOR = new Color(0.2,0.4,0.4);
    public static final double COLOR_MATCH_THRESHOLD = 0.1;

    // LIMELIGHT CONSTANTS
    public static final double A1 = 3;
    public static final double H1 = 8;
    public static final double H2 = 38;
    public static final double RADIAN_CONVERSION = 3.14159 / 180.0; // Math.toDegrees(angrad) and Math.toRadians(angdeg) are cool too
    public static final double[][] SHOOTER_DATA = new double[][] {
            // Distance, Velocity, Angle
            { 72.5, 4200, 70},
            { 36, 25, 0 },
            { 60, 30, 0 },
            { 120, 40, 0 },
            { 180, 60, 0 },
            { 240, 80, 0 },
            { 300, 95, 0 }
    };
    public static final double LIMELIGHT_ELEVATION_ANGLE = 70;
    public static final double LIMELIGHT_YAW_OFFSET = 2.5;

}
