// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;

public class TrackBallWithPixyCommand extends CommandBase {
  // instance variables
  private WestCoastDriveTrain westCoastDriveTrain;
  private PixyVisionSubsystem pixy;
  private int maxTime;
  private PIDController anglePID;
  private PIDController distancePID;
  private Timer timer;
  private int timeInTargetArea;

  /** Creates a new TrackBallWithPixyCommand. */
  public TrackBallWithPixyCommand(WestCoastDriveTrain westCoastDriveTrain, PixyVisionSubsystem pixy, int maxTime) {
    this.westCoastDriveTrain = westCoastDriveTrain;
    this.maxTime = maxTime;
    this.pixy = pixy;
    anglePID = new PIDController(Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KP, Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KI, Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KD);
    distancePID = new PIDController(Constants.TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KP, Constants.TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KI, Constants.TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KD);
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(westCoastDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    anglePID.reset();
    anglePID.setSetpoint(Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX);
    distancePID.reset();
    distancePID.setSetpoint(Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY);
    timeInTargetArea = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pixy.getFilteredX() == null){
      // There is no current pixy target
      westCoastDriveTrain.simpleArcadeDrive(0, Constants.SAFE_TURN_RATE, false);
      timeInTargetArea = 0;
      System.out.println("NO TARGET");
    }
    else{
      System.out.println("TERGET FOUND");
      // There is a current pixy target
      double x = pixy.getFilteredX();
      double y = pixy.getFilteredY();
      double linearPower = distancePID.calculate(y);
      double angularPower = anglePID.calculate(x);
      westCoastDriveTrain.simpleArcadeDrive(linearPower, -angularPower, false);

      // Update ball in target zone counter
      if(Math.abs(distancePID.getPositionError()) < Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY_TOLERANCE &&
        Math.abs(anglePID.getPositionError()) < Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX_TOLERANCE){
        timeInTargetArea ++;
      }
    }

    System.out.println("SUS");
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    westCoastDriveTrain.stopArcadeDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Timer exit condition
    if(timer.get() > maxTime){
      return true;
    }

    // Found ball exit condition
    return timeInTargetArea > Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TIME_IN_TARGET_AREA;
  }
}
