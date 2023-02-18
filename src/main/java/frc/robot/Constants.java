// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5207; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4953; 

    public static final double wheelCircumference = Units.inchesToMeters(4.0);

    public static final double angleGearRatio = (12.8 / 1.0);

    public static double falconToMeters(double positionCounts, double circumference, double GearRatio){
        return positionCounts * (circumference / (GearRatio * 2048.0));
    }


    public static final int DRIVETRAIN_PIGEON_ID = 6; 

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 17; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 16;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 56; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(232.4);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(19.4);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 19;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 34;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(204.1);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 60;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(341.1);

    public static final double kPXController = 1.35;
    public static final double kPYController = 1.75;
    public static final double kPThetaConroller = 3;
    public static final double kPSelfRight = 1.775;
    public static final double kPLimelite = 1.775;
    public static final double kPArm = 0.001; 
    public static final double kIArm = 0.0025;
    public static final double kPArmAuton = 0.9;
    public static final double kIArmAuto = 0.0002;


    public static final double slowSpeed = 0.5;
    public static final double fastSpeed = 0.85;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
    new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, 
        kMaxAngularAccelerationRadiansPerSecondSquared);

}
