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
    public static final int CURRENT_BALL_SIGNATURE = 1; // 1 = blue 2 = Red

    // Sensor Constants
    public static final int LIMELIGHT_MAX_TIME_WITH_NO_TARGET = 10;
    public static final int PIXY_MAX_TIME_WITH_NO_TARGET = 10;
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY = 180; // Target distance measurement
    public static double TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX = 150; // Target horizontal value
    public static double ROBOT_INITIAL_POS_X = -0.5; // 7.834;
    public static double ROBOT_INITIAL_POS_Y = 0; // 3.112;

    // Drive Train Constants
    // Drive Train Hardware Constants
    public static final int LEFT_DRIVE_MOTOR_1_CAN_ID = 15;
    public static final int LEFT_DRIVE_MOTOR_2_CAN_ID = 33;
    public static final int LEFT_DRIVE_MOTOR_3_CAN_ID = 2;
    public static final int RIGHT_DRIVE_MOTOR_1_CAN_ID = 11;
    public static final int RIGHT_DRIVE_MOTOR_2_CAN_ID = 13;
    public static final int RIGHT_DRIVE_MOTOR_3_CAN_ID = 5;
    public static final double FIELD_ORIENTED_DRIVE_ANGLE_KP = 0;
    public static final double FIELD_ORIENTED_DRIVE_ANGLE_KI = 0;
    public static final double FIELD_ORIENTED_DRIVE_ANGLE_KD = 0;
    public static final double FIELD_ORIENTATED_DRIVE_ANGLE_DRIVE_ZONE = 5;
    public static final double DRIVE_TRAIN_WIDTH_METERS = 0.4;
    public static final double TURRET_DEGREES_TO_TICKS_CONSTANT = 3584; // Inverse of other number
    public static final double DRIVE_TRAIN_FEED_FORWARD_KS = 0.72836;
    public static final double DRIVE_TRAIN_FEED_FORWARD_KV = 2.7874;
    public static final double DRIVE_TRAIN_FEED_FORWARD_KA = 0.28846;
    public static final double DRIVE_TRAIN_PID_KP = 0;
    public static final double DRIVE_TRAIN_PID_KI = 0;
    public static final double DRIVE_TRAIN_PID_KD = 0;
    public static final double DRIVE_TRAIN_FALCON_KP = 0;
    public static final double DRIVE_TRAIN_FALCON_KI = 0;
    public static final double DRIVE_TRAIN_FALCON_KD = 0;
    public static final double DRIVE_TRAIN_FALCON_KF = 0;
    public static final double DRIVE_TRAIN_MAX_WHEEL_SPEED_METER_PER_SECOND = 3;

    // Drive Train Control Constants
    public static final double SAFE_DRIVE_RATE = 0.3;
    public static final double SAFE_INTAKE_POWER = 0.5;
    public static double SAFE_TURN_RATE = 0.1;

    // Drive Train Turn To Angle Constants
    public static final double TURN_TO_ANGLE_KD = 0;
    public static final double TURN_TO_ANGLE_KI = 0;
    public static final double TURN_TO_ANGLE_KP = 4.2e-3; //3.8e-3


 
    // Track Ball with pixy command constants
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KP = 4.67e-3;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KI = 0;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KD = 0;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KD = 2.7e-3;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KI = 0;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KP = 0;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY_TOLERANCE = 15;
    public static final double TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX_TOLERANCE = 15;
    public static final int TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TIME_IN_TARGET_AREA = 10;

    // Drive Train Command Constants
    public static final boolean SIMPLE_WEST_COAST_DRIVE_COMMAND_SQUARE_INPUTS = true;
    public static final double ADVANCED_ARCADE_DRIVE_ANGULAR_SPEED_DEAD_ZONE = 0.1;
    public static final double ADVANCED_ARCADE_DRIVE_LINEAR_SPEED_DEAD_ZONE = 0.1;


    

    // Intake Subsystem
    public static final int INTAKE_MOTOR_CAN_ID = 10;
    public static final int TELEOP_BALL_INTAKE_JOYSTICK_AXIS = 1;
    // Turret Subsystem
    public static final int TURRET_MOTOR_CAN_ID = 7;
    public static final double TURRET_MAX_ANGLE = 75;
    public static final double TURRET_MOTOR_KP = 0.2;
    public static final double TURRET_MOTOR_KI = 0;
    public static final double TURRET_MOTOR_KD = 0;
    public static final double TURRET_MOTOR_KF = 0;
    public static final int TELEOP_CONTROL_TURRET_JOYSTICK_AXIS = 0;
    // Shooter Subsystem
    public static final int TOP_SHOOTER_MOTOR_CAN_ID = 6;
    public static final int BOTTOM_SHOOTER_MOTOR_CAN_ID = 5;
    public static final double SHOOTER_MOTOR_KP = 0.2;
    public static final double SHOOTER_MOTOR_KI = 0;
    public static final double SHOOTER_MOTOR_KD = 0;
    public static final double SHOOTER_MOTOR_KF = 0;
    public static final double MAX_SHOOTER_SPEED = 24000; //24000
    public static final int SET_SHOOTER_SPEED_TOP_SHOOTER_MOTOR_AXIS = 2;
    public static final int SET_SHOOTER_SPEED_BOTTOM_SHOOTER_MOTOR_AXIS = 0;
    public static final double[] AUTO_SHOTER_SPEED_BOTTOM_MOTOR_SPEEDS = {-10830, -12380, -12390, -10990, -13610};//{-10070, -10422, -10154, -10125, -10723, -9999};
    public static final double[] AUTO_SHOOTER_SPEED_TOP_MOTOR_SPEEDS = {4930, 4930, 4470, 8020, 6170};//{4580, 4911, 7077, 14266, 16470, 16830};
    public static final double[] AUTO_SHOOTER_SPEED_RANGES = {12.39, -2.5, 2.8, -4.6, -8.5};//{14.6, 11.4, 6.6, 3.9, 2.6, 0.7};

    // Climber Constants
    public static final int CLIMBER_MOTOR_CAN_ID = 12;

    // Unit Conversion Constants
    public static final double FALCON_TICKS_TO_METERS = 1.0 / 2048.0 * 1.0 / 12.75 * 0.4785;
    public static final double FALCON_VELOCITY_TO_METERS_PER_SECOND = 1.948e-4 / 12.75;
    public static final double TURRET_TICKS_TO_DEGREES_CONSTANT =  2.790178571e-4; //1 * (1 / 2048) * (1 / 45) * (1 / 14) * ( 360 / 1);
    // Convert from ticks to motor rotations (1 motor rotation per 2048 ticks) to turret rotations 
    // (1 turret rotation per 45 motor rotations) degrees (360 degrees per rotation)
   
    // Trajectory Constants
    public static final double TRAJECTORY_MAX_SPEED = 1;
    public static final double TRAJECTORY_MAX_ACCELERATION = 0.25;

    // Tracking Constants
    public static final double TRACK_TARGET_WITH_ROBOT_KP = 1e-2;
    public static final double TRACK_TARGET_WITH_ROBOT_KI = 0;
    public static final double TRACK_TARGET_WITH_ROBOT_KD = 0;
    public static final double TRACK_TARGET_WITH_ROBOT_TOLERANCE = 10;
    public static final int TRACK_TARGET_WITH_ROBOT_MAX_TIME_IN_TOLERANCE = 0;
    public static final double TRACK_TARGET_WITH_ODOMETRY_LIMELIGHT_ZONE = 0;







    // Field Cordinates Constants
    public static double FIELD_CENTER_X_CORD = 0; // 8.2296;
    public static double FIELD_CENTER_Y_CORD = 0; //4.1148;
    public static double FIRST_BALL_X_CORD = 0; // 5.136;
    public static double FIRST_BALL_Y_CORD = 0; // 1.848;
    public static double SECOND_BALL_X_CORD = 0; // 1.094;
    public static double SECOND_BALL_Y_CORD = 0; // 1.131;
 










    

    
}
