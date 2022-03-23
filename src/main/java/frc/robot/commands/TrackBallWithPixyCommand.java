// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;
import io.github.pseudoresonance.pixy2api.Pixy2;

public class TrackBallWithPixyCommand extends CommandBase {
  // instance variables
  private int signature;
  private WestCoastDriveTrain westCoastDriveTrain;
  private Pixy2 pixy;
  private int maxTime;
  private PixyVisionSubsystem pixyVisionSubsystem;
  private PIDController anglePID;
  private PIDController distancePID;

  /** Creates a new TrackBallWithPixyCommand. */
  public TrackBallWithPixyCommand(WestCoastDriveTrain westCoastDriveTrain, Pixy2 pixy, int signature, int maxTime, PixyVisionSubsystem pixyVisionSubsystem) {
    this.westCoastDriveTrain = westCoastDriveTrain;
    this.signature = signature;
    this.maxTime = maxTime;
    this.pixy = pixy;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pixyVisionSubsystem.getFilteredX() == null){
      westCoastDriveTrain.simpleArcadeDrive(0, Constants.SAFE_TURN_RATE, false);
    }
    else{

    }

  }

  public void followBall(){


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
