// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WestCoastDriveTrain;

/**
 * A command to drive the robot using the Advanced Arcade Drive
 * Methods
 */
public class AdvancedWestCoastDriveCommand extends CommandBase {
  // Instance Variables
  private WestCoastDriveTrain driveTrain;
  private Joystick joystick;
  private boolean squareInputs;

  /** Creates a new AdvancedWestCoastDriveCommand. */
  public AdvancedWestCoastDriveCommand(WestCoastDriveTrain driveTrain, Joystick joystick, boolean squareInputs){
    this.driveTrain = driveTrain;
    this.joystick = joystick;
    this.squareInputs = squareInputs;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.stopArcadeDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(joystick.getRawAxis(5)) > 0.07 || Math.abs(joystick.getRawAxis(4)) > 0.07){
      driveTrain.advancedArcadeDrive(-joystick.getRawAxis(5), joystick.getRawAxis(4), squareInputs);
    }
    else{
      driveTrain.stopArcadeDrive();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopArcadeDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
