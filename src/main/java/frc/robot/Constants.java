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
    // Alignment Constants
    // The filepath on the RIO for the wheel alignment constants
    public static final String ALIGNMENT_CONSTANTS_FILE_PATH = "home/lvuser/WheelConstants";

    // User Input Constants
    public static final int JOYSTICK_0_PORT_NUMBER = 0;
    public static final int CURRENT_BALL_SIGNATURE = 0;

    // Sensor Constants
    public static final int LIMELIGHT_MAX_TIME_WITH_NO_TARGET = 10;
    public static final int PIXY_MAX_TIME_WITH_NO_TARGET = 10;

    // Drive Train Constants
    // Drive Train Hardware Constants
    public static final int LEFT_DRIVE_MOTOR_1_CAN_ID = 0;
    public static final int LEFT_DRIVE_MOTOR_2_CAN_ID = 1;
    public static final int LEFT_DRIVE_MOTOR_3_CAN_ID = 2;
    public static final int RIGHT_DRIVE_MOTOR_1_CAN_ID = 3;
    public static final int RIGHT_DRIVE_MOTOR_2_CAN_ID = 4;
    public static final int RIGHT_DRIVE_MOTOR_3_CAN_ID = 5;
    public static final double DRIVE_MOTOR_KP = 0;
    public static final double DRIVE_MOTOR_KI = 0;
    public static final double DRIVE_MOTOR_KD = 0;
    public static final double DRIVE_MOTOR_KF = 0;
    public static final double FIELD_ORIENTED_DRIVE_ANGLE_KP = 0;
    public static final double FIELD_ORIENTED_DRIVE_ANGLE_KI = 0;
    public static final double FIELD_ORIENTED_DRIVE_ANGLE_KD = 0;
    public static final double FIELD_ORIENTATED_DRIVE_ANGLE_DRIVE_ZONE = 5;
    public static final double DRIVE_TRAIN_WIDTH_METERS = 0.4;


    // Drive Train Command Constants
    public static final boolean SIMPLE_WEST_COAST_DRIVE_COMMAND_SQUARE_INPUTS = true;

    

    // Intake Subsystem
    public static final int INTAKE_MOTOR_CAN_ID = 6;
    public static final int TELEOP_BALL_INTAKE_JOYSTICK_AXIS = 4;
    // Turret Subsystem
    public static final int TURRET_MOTOR_CAN_ID = 7;
    public static final double TURRET_MAX_ANGLE = 100;
    public static final double TURRET_MOTOR_KP = 0;
    public static final double TURRET_MOTOR_KI = 0;
    public static final double TURRET_MOTOR_KD = 0;
    public static final double TURRET_MOTOR_KF = 0;
    public static final int TELEOP_CONTROL_TURRET_JOYSTICK_AXIS = 6;
    // Shooter Subsystem
    public static final int TOP_SHOOTER_MOTOR_CAN_ID = 8;
    public static final int BOTTOM_SHOOTER_MOTOR_CAN_ID = 9;
    public static final double SHOOTER_MOTOR_KP = 0;
    public static final double SHOOTER_MOTOR_KI = 0;
    public static final double SHOOTER_MOTOR_KD = 0;
    public static final double SHOOTER_MOTOR_KF = 0;
    public static final int SET_SHOOTER_SPEED_TOP_SHOOTER_MOTOR_AXIS = 0;
    public static final int SET_SHOOTER_SPEED_Bottom_SHOOTER_MOTOR_AXIS = 0;

    // Unit Conversion Constants
    public static final double FALCON_TICKS_TO_METERS = 2048.0 * 0.1; 
    public static final double TURRET_TICKS_TO_DEGREES_CONSTANT =  2.790178571e-4; //1 * (1 / 2048) * (1 / 45) * (1 / 14) * ( 360 / 1);
    // Convert from ticks to motor rotations (1 motor rotation per 2048 ticks) to turret rotations 
    // (1 turret rotation per 45 motor rotations) degrees (360 degrees per rotation)
    public static final double TURRET_DEGREES_TO_TICKS_CONSTANT = 3584; // Inverse of other number

    public static final double DRIVE_TRAIN_FEED_FORWARD_KS = 0;

    public static final double DRIVE_TRAIN_FEED_FORWARD_KV = 0;

    public static final double DRIVE_TRAIN_FEED_FORWARD_KA = 0;

    public static final double DRIVE_TRAIN_PID_KP = 0;

    public static final double DRIVE_TRAIN_PID_KI = 0;

    public static final double DRIVE_TRAIN_PID_KD = 0;

    public static final double TRAJECTORY_MAX_SPEED = 0;

    public static final double TRAJECTORY_MAX_ACCELERATION = 0;









    

    
}
