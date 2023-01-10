// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DEGREES_PER_PULSE = 360.0/4096.0;
    public static final double METERS_PER_PULSE = 0.00002226;
    public static final double MAX_VELOCITY = .5;//4.8;

    //Swerve Module PID Numbers
    public static final double SWERVE_MODULE_DRIVE_P = 0;//1000;
    public static final double SWERVE_MODULE_DRIVE_I = 0;
    public static final double SWERVE_MODULE_DRIVE_D = 0;//25;
    public static final double SWERVE_MODULE_ROTATION_P = 8;
    public static final double SWERVE_MODULE_ROTATION_I = 0.0;//0.2;
    public static final double SWERVE_MODULE_ROTATION_D = 2500;
    public static final double SWERVE_MODULE_ROTATION_I_ZONE = 0;//10/Constants.DEGREES_PER_PULSE;

    public static final double DRIVE_ROTATION_CONTROLLER_P = 1.1;// 8;
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = .001;//0.2;
    public static final double DRIVE_ROTATION_CONTROLLER_F = 0;

    public static final double DRIVE_MAX_ANGULAR_ACCEL = 600;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 300;

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(Units.inchesToMeters(21.25/2.0), Units.inchesToMeters(25.0/2.0));
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(Units.inchesToMeters(21.25/2.0), -Units.inchesToMeters(25.0/2.0));
    public static final Translation2d REAR_LEFT_OFFSET = new Translation2d(-Units.inchesToMeters(21.25/2.0), Units.inchesToMeters(25.0/2.0));
    public static final Translation2d REAR_RIGHT_OFFSET = new Translation2d(-Units.inchesToMeters(21.25/2.0), -Units.inchesToMeters(25.0/2.0));

    //Device ID's

    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int REAR_LEFT_DRIVE_ID = 1;
    public static final int REAR_RIGHT_DRIVE_ID = 3;
  
    public static final int FRONT_LEFT_ROTATION_ID = 6;
    public static final int FRONT_RIGHT_ROTATION_ID = 8;
    public static final int REAR_LEFT_ROTATION_ID = 5;
    public static final int REAR_RIGHT_ROTATION_ID = 7;
}