// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbingSubsystem;

public class UseClimberCommand extends CommandBase {
  private ClimbingSubsystem climber;
  private Joystick joystick;
  private int axis;
  /** Creates a new UseClimberCommand. */
  public UseClimberCommand(ClimbingSubsystem climber, Joystick joystick, int axis) {
    this.climber = climber;
    this.joystick = joystick;
    this.axis = axis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = joystick.getRawAxis(axis);
    if(Math.abs(power) > 0.05){
      climber.powerClimber(-power);
    }
    else{
      climber.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
