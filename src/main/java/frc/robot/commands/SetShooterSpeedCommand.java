// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PursellJaques.ShooterSpeeds;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterSpeedCommand extends CommandBase {
  //instance Variables
  private ShooterSubsystem shooterSubsystem;
  private Joystick joystick;
  private Joystick sj;
  private int topMotorAxis;
  private int bottomMotorAxis;
  /** Creates a new SetShooterSpeedCommand. */
  public SetShooterSpeedCommand(int topMotorAxis, int bottomMotorAxis, Joystick joystick, Joystick sj, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.topMotorAxis = topMotorAxis;
    this.bottomMotorAxis = bottomMotorAxis;
    this.joystick = joystick;
    this.shooterSubsystem = shooterSubsystem;
    this.sj = sj;

    addRequirements(shooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.simpleSetShooterSpeeds(new ShooterSpeeds(joystick.getRawAxis(Constants.SET_SHOOTER_SPEED_TOP_SHOOTER_MOTOR_AXIS) * Constants.MAX_SHOOTER_SPEED,  joystick.getRawAxis(Constants.SET_SHOOTER_SPEED_TOP_SHOOTER_MOTOR_AXIS) * Constants.MAX_SHOOTER_SPEED));
    // System.out.println(joystick.getRawAxis(Const));
    // shooterSubsystem.simpleSetShooterSpeeds(new ShooterSpeeds(
    //   sj.getRawAxis(topMotorAxis) * Constants.MAX_SHOOTER_SPEED, 
    //   joystick.getRawAxis(3) * Constants.MAX_SHOOTER_SPEED));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    return false;
  }
}
