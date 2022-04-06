// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;

/**
 * A command that has the robot drive forward with the intake active
 */
public class SimpleAutoDriveAndIntakeCommand extends CommandBase {
  private Timer intakeTimer;
  private Timer driveTimer;
  private double intakeTime;
  private double driveTime;
  private WestCoastDriveTrain driveTrain;
  private IntakeSubsystem intake; 
  private boolean reversed;

  /** Creates a new AutoDriveAndIntakeCommand. 
   * @param intakeTime The time in seconds that the robot should drive forward
   * @param driveTime The time in seconds that the robot should intake
   * @param reversed True if the robot should drive backward
  */
  public SimpleAutoDriveAndIntakeCommand(WestCoastDriveTrain driveTrain, IntakeSubsystem intake, double intakeTime, double driveTime, boolean reversed) {
    this.intake = intake;
    this.driveTrain = driveTrain;
    this.driveTime = driveTime;
    this.intakeTime = intakeTime;
    intakeTimer = new Timer();
    driveTimer = new Timer();
    this.reversed = reversed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.start();
    driveTimer.reset();
    driveTimer.start();
    driveTrain.driveWithVoltage(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive the robot until the timer is past the time
    if(driveTimer.get() < driveTime){
      if(reversed == true){
        driveTrain.simpleArcadeDrive(-Constants.SAFE_DRIVE_RATE, 0, false);
      }
      else{
        driveTrain.simpleArcadeDrive(Constants.SAFE_DRIVE_RATE, 0, false);
      }
    }
    else{
      driveTrain.simpleArcadeDrive(0, 0, false);
    }
    // Power the intake until the timer is past the time
    if(intakeTimer.get() < intakeTime){
      intake.driveIntake(Constants.SAFE_INTAKE_POWER);
    }
    else{
      intake.driveIntake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.driveIntake(0);
    driveTrain.simpleArcadeDrive(0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when both timers are over
    return driveTimer.get() > driveTime && intakeTimer.get() > intakeTime;
  }
}
