// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WestCoastDriveTrain;

public class SimpleTurnRobot extends CommandBase {
  private WestCoastDriveTrain driveTrain;
  private double angle;
  private double targetAngle;
  private int timeInTarget;
  private Timer timer;
  private double maxTime;

  /** Creates a new SimpleTurnRobot. */
  public SimpleTurnRobot(WestCoastDriveTrain driveTrain, double angle, double maxTime) {
    this.angle = angle;
    this.driveTrain = driveTrain;
    this.maxTime = maxTime;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = driveTrain.getAngle() + angle;
    timeInTarget = 0;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveTrain.getAngle() < targetAngle - 0.1 * angle){
      driveTrain.simpleArcadeDrive(0, Constants.SAFE_TURN_RATE, false);
      timeInTarget = 0;

    }
    else if(driveTrain.getAngle() > targetAngle + 0.1 * angle){
      driveTrain.simpleArcadeDrive(0, -Constants.SAFE_TURN_RATE, false);
      timeInTarget = 0;
    }
    else{
      timeInTarget ++;
      driveTrain.stopArcadeDrive();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > maxTime || timeInTarget > 20;
  }
}
