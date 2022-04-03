// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PursellJaques.TargetTrackingType;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;

public class TrackTargetWithOdometryCommand extends CommandBase {
  // Instance Variables
  private TurretSubsystem turret;
  private LimelightVisionSubsystem limelight;
  private WestCoastDriveTrain driveTrain;
  private double limelightZone; 
  /**
   * The range of degrees in which the limelight will be used. 
   * When the difference between the odometry-calculated angle and the curret turret
   * angle is less than this number, full controll will be given to the limelight
  */

  /** Creates a new TrackTargetWithOdometry. 
   * @param turret The turret subsystem that the command will control
   * @param limelight The limelight that the command will use to track the target
   * @param driveTrain The drive train the command will use to get odometry data (will not actually drive it)
   * @param limelightZone The range of degrees in which the limelight will be used. 
   * When the difference between the odometry-calculated angle and the curret turret
   * angle is less than this number, full controll will be given to the limelight
  */
  public TrackTargetWithOdometryCommand(TurretSubsystem turret, LimelightVisionSubsystem limelight, 
    WestCoastDriveTrain driveTrain, double limeLighZone) {
    
    this.turret = turret;
    this.limelight = limelight;
    this.driveTrain = driveTrain;
    this.limelightZone = limeLighZone;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    /**
     * While we are using the limelight and drive train as well, we are not going to be giving these subsystems
     * any commands that would interfere with their normal use, so we will not add them as a requirement.
     * In fact, adding them as a requirement would be bad, because the robot would not be able to drive and
     * use this command in parallel
     */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate angle to goal
    double deltaX = Constants.FIELD_CENTER_X_CORD - driveTrain.getPose().getX();
    double deltaY = Constants.FIELD_CENTER_Y_CORD - driveTrain.getPose().getY();
    double absoluteAngleToTarget = Math.toDegrees(Math.atan2(deltaX, deltaY));
    // Determine whether to use odometry or limelight to track the target
    TargetTrackingType trackingType;
    if(limelight.getFilteredV() == 0){
      // If the limelight has no target, use the odometry
      trackingType = TargetTrackingType.Odometry;
    }
    else{
      // There is a valid target
      // Calculate, then convert to degrees
      // Calculate current target angle from the robots angle, the turret's angle, and the target's angle in the limelight's field of view
      double currentDirection = -driveTrain.getHeading().getDegrees() + turret.getCurrentAngle() + limelight.getFilteredX();
      // Calculate angle difference
      double angleDifference = absoluteAngleToTarget - currentDirection;
      
      if(Math.abs(angleDifference) > limelightZone){
        // Limelight sees target, but the combined angle from the turret's turn and the targets position in the 
        // turrets field of vision are too far outside the expected limelight zone
        trackingType = TargetTrackingType.Odometry;
      }
      else{
        // Limelight sees target, and target is inside the allowed limelight zone
        trackingType = TargetTrackingType.Limelight;
      }
    }

    // Power the turret based on the tracking type
    if(trackingType == TargetTrackingType.Odometry){
      // Track using Odometry
      turret.setTurretAbsoluteAngle(absoluteAngleToTarget + driveTrain.getHeading().getDegrees());
      // Set the relative angle to the absolute angle - the robot's current angle
    }
    else if(trackingType == TargetTrackingType.Limelight){
      // Track using Limelight
      turret.setTurretRelativeAngle(limelight.getFilteredX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
