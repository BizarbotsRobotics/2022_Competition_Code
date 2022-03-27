// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /* 
    * General Constants
    */

    public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
    



    /**
     * Drive Constants
     */

    public static final double PIGEON_OFFSET = 12;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5588; // FIXME Measure and set trackwidth

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5842; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set front left module drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 23; // FIXME Set front left steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(339.08); // FIXME Measure and set front left steer offset

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front LEFT drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front LEFT steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 22; // FIXME Set front LEFT steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(13.45); // FIXME Measure and set front LEFT steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back left drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back left steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 21; // FIXME Set back left steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(59.85); // FIXME Measure and set back left steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set back LEFT drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set back LEFT steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 24; // FIXME Set back LEFT steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(107.49); // FIXME Measure and set back LEFT steer offset

    /* 
    * Intake Constants 
    */

    public static final int[] INTAKE_SOLENOID_PORTS = {0,1};
    public static final int INTAKE_WHEEL_ID = 10;
    public static final double INTAKE_WHEEL_SPEED = .8;
    public static final double OUTTAKE_WHEEL_SPEED = .6;

    /* 
    * Conveyor Constants
    */
    public static final int TOP_CONVEYOR_ID = 10;
    public static final double TOP_CONVEYOR_SPEED = .7;
    public static final int BOTTOM_CONVEYOR_ID = 15;
    public static final double BOTTOM_CONVEYOR_SPEED = .6;
    
    public static final double CONVEYOR_kP = 0.0001; 
    public static final double CONVEYOR_kI = 0;
    public static final double CONVEYOR_kD = 0; 
    public static final double CONVEYOR_kIz = 0; 
    public static final double CONVEYOR_kFF = 0.00019; 
    public static final double CONVEYOR_kMaxOutput = 1; 
    public static final double CONVEYOR_kMinOutput = -1;
    public static final double CONVEYOR_maxRPM = 5400;
    public static final int CONVEYOR_CURRENT_LIMIT = 20;

    public static final int DEFAULT_CONVEYOR_DISTANCE = 10;
    public static final int CONVEYOR_PROXIMITY_THRESHOLD = 200;
    public static final int CONVEYOR_COLOR_THRESHOLD = 600;
    /*
    * Sushi Constants
    */

    public static final int SUSHI_ID = 9;
    public static final double SUSHI_SPEED = -.6;

    /*
    * Shooter Constants
    */
    public static final int SHOOTER_DEFAULT_SPEED = 2500;
    public static final int[] SHOOTER_SOLENOID_PORTS = {6,7};
    public static final int SHOOTER_ID_LEFT = 12;
    public static final int SHOOTER_ID_RIGHT = 11;

    public static final double SHOOTER_kP = 0.0006; 
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0; 
    public static final double SHOOTER_kIz = 0; 
    public static final double SHOOTER_kFF = 0.000183; 
    public static final double SHOOTER_kMaxOutput = 1; 
    public static final double SHOOTER_kMinOutput = -1;
    public static final double SHOOTER_maxRPM = 5400;
    public static final int SHOOTER_CURRENT_LIMIT = 40;

    public static final int SHOOTER_SPEED_RANGE = 40;
    public static final double SHOOTER_REJECT_PERCENTAGE = .6;

    /*
    * Climber Constants
    */
    public static final int CLIMBER_ID_RIGHT = 9;
    public static final int CLIMBER_ID_LEFT = 16;
    public static final int CLIMBER_ID_AUX = 15;
    public static final double CLIMBER_SPEED = 1;
    public static final double AUX_ARM_SPEED = .5;


    /* 
    * LIMELIGHT PID VALUES
    */

    public static final double GOAL_ALIGN_KP = 0.008;
    public static final double GOAL_ALIGN_KD = 0.0008;
}

