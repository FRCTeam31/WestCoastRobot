// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WestCoastDriveTrain;

/**
 * Aim the robot towards the expected position of the target usng odometry
 */
public class TrackTargetWithRobotUsingOdometryCommand extends CommandBase {
  // Instance Varables
  private WestCoastDriveTrain driveTrain;
  private PIDController anglePIDController;
  private Timer timer;
  private double maxTime;
  private int timeInTolerance;

  /** Creates a new TrackTargetWithRobotUsingOdometryCommand.
   * @param driveTrain The driveTrain that the command will control
   * @param maxTime The maximum time allowed for this command
   */
  public TrackTargetWithRobotUsingOdometryCommand(WestCoastDriveTrain driveTrain, double maxTime) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    anglePIDController = new PIDController(Constants.TRACK_TARGET_WITH_ROBOT_KP, Constants.TRACK_TARGET_WITH_ROBOT_KI, Constants.TRACK_TARGET_WITH_ROBOT_KD);
    anglePIDController.enableContinuousInput(0, 360);
    this.maxTime = maxTime;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePIDController.reset();
    anglePIDController.setTolerance(Constants.TRACK_TARGET_WITH_ROBOT_TOLERANCE);
    timer.reset();
    timer.start();
    timeInTolerance = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate angle to goal
    double deltaX = Constants.FIELD_CENTER_X_CORD - driveTrain.getPose().getX();
    double deltaY = Constants.FIELD_CENTER_Y_CORD - driveTrain.getPose().getY();
    double absoluteAngleToTarget = Math.toDegrees(Math.atan2(deltaX, deltaY));
    // Power Turning based on PID output
    driveTrain.simpleArcadeDrive(0.0, anglePIDController.calculate(absoluteAngleToTarget, driveTrain.getHeading().getDegrees()), false);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopArcadeDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Update Counter
    if(Math.abs(anglePIDController.getPositionError()) < Constants.TRACK_TARGET_WITH_ROBOT_TOLERANCE){
      timeInTolerance ++;
    }
    else{
      timeInTolerance = 0;
    }

    // Exit if time is up
    if(timer.get() > maxTime){
      return true;
    }

    // Exit if the obot has been inside tolerance zone for an amount of time
    if(timeInTolerance > Constants.TRACK_TARGET_WITH_ROBOT_MAX_TIME_IN_TOLERANCE){
      return true;
    }

    // Otherwise, return false
    return false;
  }
}
