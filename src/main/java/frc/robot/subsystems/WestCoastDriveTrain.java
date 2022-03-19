// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

/**
 * A class to manage the drivetrain of the robot (West coast)
 * This implementation uses Falcon500
 */
public class WestCoastDriveTrain extends SubsystemBase {
  // Instance Variables
  // Drive Variables
  private WPI_TalonFX[] leftMotors;
  private WPI_TalonFX[] rightMotors;
  private DifferentialDrive differentialDrive;
  // Sensor Variables
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry driveOdometry;
  private AHRS navx;
  // Motion Controllers
  private PIDController fieldOrientedDriveAnglePIDController;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DRIVE_TRAIN_FEED_FORWARD_KS, Constants.DRIVE_TRAIN_FEED_FORWARD_KV, Constants.DRIVE_TRAIN_FEED_FORWARD_KA);
  private PIDController leftPIDController = new PIDController(Constants.DRIVE_TRAIN_PID_KP, Constants.DRIVE_TRAIN_PID_KI, Constants.DRIVE_TRAIN_PID_KD);
  private PIDController rightPIDController = new PIDController(Constants.DRIVE_TRAIN_PID_KP, Constants.DRIVE_TRAIN_PID_KI, Constants.DRIVE_TRAIN_PID_KD);

  /** Creates a new WestCoastDriveTrain. 
   * @param leftMotors list of motors controlling the left side of the drive train
   * @param rightMotors list of motors controlling the right side of the drive train
   * @param navx The navx object for the drivetrain to use
   * @param trackWidthMeters The width of the drive train in meters (distance from left side to right side)
  */
  public WestCoastDriveTrain(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, AHRS navx, double trackWidthMeters) {
    // Drive Variables
    this.leftMotors = leftMotors;
    this.rightMotors = rightMotors;

    // Configure proper motor inversions
    for(int index = 0; index < leftMotors.length; index ++){
      if(index == 0){
        this.rightMotors[index].setInverted(TalonFXInvertType.Clockwise);
        this.leftMotors[index].setInverted(TalonFXInvertType.CounterClockwise);
      }
      else{
        this.rightMotors[index].follow(this.rightMotors[0]);
        this.rightMotors[index].setInverted(InvertType.FollowMaster);
        this.leftMotors[index].follow(this.rightMotors[0]);
        this.leftMotors[index].setInverted(InvertType.FollowMaster);
      }
    }

    // Sensor Variables   
    for(WPI_TalonFX talon: this.leftMotors){
      talon.setSelectedSensorPosition(0);
    }
    for(WPI_TalonFX talon: this.rightMotors){
      talon.setSelectedSensorPosition(0);
    }

    this.navx = navx;
    this.navx.reset();
    kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    driveOdometry = new DifferentialDriveOdometry(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the robot's current position
    double leftSensor = leftMotors[0].getSelectedSensorPosition() * FALCON_TICKS_TO_METERS;
    double rightSensor = rightMotors[0].getSelectedSensorPosition() * FALCON_TICKS_TO_METERS;
    driveOdometry.update(getHeading(), leftSensor, rightSensor);

    Pose2d currentPos = getPose();
    // Display current position
    SmartDashboard.putNumber("X-Cord", currentPos.getX());
    SmartDashboard.putNumber("Y-Cord", currentPos.getY());
    SmartDashboard.putNumber("Rotation", currentPos.getRotation().getDegrees());
  }

  /**
   * Drive using a simple arcade drive command
   * @param linearSpeed the speed of forwards & backwards motion, in the range of [-1, 1]
   * @param angularSpeed the speed of the clockwise & counter-clockwise motion, in the range of [-1, 1]
   * @param squareInputs if true, square inputs in order to make controls more sensitive at low values
   */
  public void simpleArcadeDrive(double linearSpeed, double angularSpeed, boolean squareInputs){
    differentialDrive.arcadeDrive(linearSpeed, angularSpeed, squareInputs);
  }

  /**
   * Drive the robot using voltages
   * @param leftVoltage The voltage to send to the left side
   * @param rightVoltage The voltage to send to the right side
   */
  public void driveWithVoltage(double leftVoltage, double rightVoltage){
    leftMotors[0].setVoltage(leftVoltage);
    rightMotors[0].setVoltage(rightVoltage);
  }

  /**
   * Reset PID controller for Field Orientated Driving
   */
  public void instantiateFieldOrientatedDrive(){
    fieldOrientedDriveAnglePIDController = new PIDController(
      Constants.FIELD_ORIENTED_DRIVE_ANGLE_KP,
      Constants.FIELD_ORIENTED_DRIVE_ANGLE_KI,
      Constants.FIELD_ORIENTED_DRIVE_ANGLE_KD);
    fieldOrientedDriveAnglePIDController.enableContinuousInput(0, 360);
  }

  /**
   * Field Orientated driving, that powers the linear speed based on how much the robot is 
   * alighned with the target angle
   * @param power Ideal power for the robot to drive
   * @param targetAngle Target angle for the robot to be aimed torwards in degrees
   */
  public void fieldOreintatedNoah(double power, double targetAngle){
    double currentAngle = (navx.getAngle() % 360 + 360) % 360;
    double angularSpeed = fieldOrientedDriveAnglePIDController.calculate(currentAngle, targetAngle);

    double angleDifference = ((targetAngle - currentAngle) % 360 + 360) % 360;
    double linearSpeed;
    if(angleDifference < Constants.FIELD_ORIENTATED_DRIVE_ANGLE_DRIVE_ZONE
    || angleDifference > (360 - Constants.FIELD_ORIENTATED_DRIVE_ANGLE_DRIVE_ZONE)){
      linearSpeed = Math.cos(Math.toRadians(angleDifference));
    }
    else {
      linearSpeed = 0;
    }

    this.simpleArcadeDrive(linearSpeed, angularSpeed, false);
    
  }


  /**
   * Field orientated driving where the robot uses linear speed when the angle is 
   * inside a target range
   * @param power Ideal power for the robot to drive 
   * @param targetAngle Target angle for the robot to be aimed torwards
   */
  public void fieldOrientedZach(double power,  double targetAngle){
    double currentAngle = (navx.getAngle() %  360 + 360) % 360;
    double angularSpeed = fieldOrientedDriveAnglePIDController.calculate(
      currentAngle,
      targetAngle);
    double linearSpeed = 0;
    double angleDifference = ((targetAngle - currentAngle) % 360 + 360) % 360;
    if(angleDifference < Constants.FIELD_ORIENTATED_DRIVE_ANGLE_DRIVE_ZONE
    || angleDifference > (360 - Constants.FIELD_ORIENTATED_DRIVE_ANGLE_DRIVE_ZONE)){
      linearSpeed = power;
    }
    this.simpleArcadeDrive(linearSpeed, angularSpeed, false);
  }

  /**
   * Stop the arcade drive's motors
   */
  public void stopArcadeDrive(){
    differentialDrive.stopMotor();
  }

  /**
   * 
   * @return The wheels speeds in meters/second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      leftMotors[0].getSelectedSensorVelocity() * Constants.FALCON_TICKS_TO_METERS * 10,
      rightMotors[0].getSelectedSensorVelocity() * Constants.FALCON_TICKS_TO_METERS * 10
    );
  }

  /**
   * 
   * @return The angle the robot is facing
   */
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  /**
   * 
   * @return The current position of the robot
   */
  public Pose2d getPose(){
    return driveOdometry.getPoseMeters();
  }

  /**
   * 
   * @return This object's feed forward controller
   */
  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  /**
   * 
   * @return The left motor's PID controller
   */
  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  /**
   * 
   * @return The right motor's PID
   */
  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  /**
   * 
   * @return The object's kinematics class
   */
  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }
  
}
