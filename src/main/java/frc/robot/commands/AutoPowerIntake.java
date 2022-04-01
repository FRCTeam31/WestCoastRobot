// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * A command that powers the intake for a specified time
 */
public class AutoPowerIntake extends CommandBase {
  double intakeTime;
  Timer intakeTimer;
  IntakeSubsystem intake;
  /** Creates a new AutoPowerIntake. 
   * @param intake The intake for the command to control
   * @param intakeTime The time in seconds that the intake should be powered
  */
  public AutoPowerIntake(IntakeSubsystem intake, double intakeTime) {
    this.intake = intake;
    this.intakeTime = intakeTime;
    intakeTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.driveIntake(0);
    intakeTimer.reset();
    intakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.driveIntake(Constants.SAFE_INTAKE_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when the timer is past the designated time
    return intakeTimer.get() > intakeTime;
  }
}
