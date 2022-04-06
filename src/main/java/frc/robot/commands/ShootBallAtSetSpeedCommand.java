// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PursellJaques.ShooterSpeeds;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBallAtSetSpeedCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;
  private Timer timer;
  private double shootingTime;
  private double intakeTime;
  private double shootingPower;
  /** Creates a new ShootBallAtSetSpeedCommand. */
  public ShootBallAtSetSpeedCommand(ShooterSubsystem shooter, IntakeSubsystem intake, double shootingTime, double intakeTime, double shootingPower) {
    this.shooter = shooter;
    this.intake = intake;
    this.shootingTime = shootingTime;
    this.intakeTime = intakeTime;
    this.timer = new Timer();
    this.shootingPower = shootingPower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < shootingTime){
      shooter.simpleSetShooterSpeeds(new ShooterSpeeds(shootingPower, shootingPower));
    }
    else{
      shooter.stopMotion();
    }
    if(timer.get() < intakeTime){
      intake.driveIntake(Constants.SAFE_INTAKE_POWER);
    }
    else{
      intake.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    shooter.stopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > shootingTime && timer.get() > intakeTime;
    
  }
}
