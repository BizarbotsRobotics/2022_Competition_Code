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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5588; // FIXME Measure and set trackwidth

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5842; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 4; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(60.82031421915981); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(110.74218735335052); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(333.1933480163102); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(359.472692093231); // FIXME Measure and set back right steer offset

    /* 
    * Intake Constants 
    */

    public static final int[] INTAKE_SOLENOID_PORTS = {0,1};
    public static final int INTAKE_WHEEL_ID = 10;
    public static final double INTAKE_WHEEL_SPEED = .6;

    /* 
    * Conveyor Constants
    */
    public static final int TOP_CONVEYOR_ID = 10;
    public static final double TOP_CONVEYOR_SPEED = .6;
    public static final int BOTTOM_CONVEYOR_ID = 15;
    public static final double BOTTOM_CONVEYOR_SPEED = .6;
    
    /*
    * Sushi Constants
    */

    public static final int SUSHI_ID = 9;
    public static final double SUSHI_SPEED = -.6;

    /*
    * Shooter Constants
    */
    public static final int[] SHOOTER_SOLENOID_PORTS = {0,1};
    public static final int SHOOTER_ID_LEFT = 12;
    public static final int SHOOTER_ID_RIGHT = 11;

    public static final double SHOOTER_kP = 6e-5; 
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0; 
    public static final double SHOOTER_kIz = 0; 
    public static final double SHOOTER_kFF = 0.000015; 
    public static final double SHOOTER_kMaxOutput = 1; 
    public static final double SHOOTER_kMinOutput = -1;
    public static final double SHOOTER_maxRPM = 5700;
    public static final int SHOOTER_CURRENT_LIMIT = 20;
}

