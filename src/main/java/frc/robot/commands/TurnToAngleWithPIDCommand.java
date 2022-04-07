// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WestCoastDriveTrain;

public class TurnToAngleWithPIDCommand extends CommandBase {
  //instance variables
  private WestCoastDriveTrain driveTrain;
  private PIDController anglePID;
  private int counter;
  private Timer timer;
  private double angle;
  private double maxTime;
  /** Creates a new TurnToAngleWithPIDCommand. */
  public TurnToAngleWithPIDCommand(WestCoastDriveTrain driveTrain, double angle, double maxTime) {
    this.driveTrain = driveTrain;
    this.angle = angle;
    this.maxTime = maxTime;
    anglePID = new PIDController(Constants.TURN_TO_ANGLE_KP, Constants.TURN_TO_ANGLE_KI, Constants.TURN_TO_ANGLE_KD);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    timer.reset();
    timer.start();
    anglePID.setSetpoint(driveTrain.getAngle() + angle);
    anglePID.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.advancedArcadeDrive(0, anglePID.calculate(driveTrain.getAngle()), false);
    if(Math.abs(anglePID.getPositionError()) < 5){
      counter++;
    }
    else{
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > maxTime || counter > 10){
      return true;
    }
    else{
      return false;
    }
  }
}
