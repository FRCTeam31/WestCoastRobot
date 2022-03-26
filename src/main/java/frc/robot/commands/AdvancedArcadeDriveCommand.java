// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.WestCoastDriveTrain;

public class AdvancedArcadeDriveCommand extends CommandBase {
  //instance variables
  WestCoastDriveTrain westCoastDriveTrain;
  Joystick joystick;
  /** Creates a new AdvancedArcadeDriveCommand. */
  public AdvancedArcadeDriveCommand(WestCoastDriveTrain westCoastDriveTrain, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.westCoastDriveTrain = westCoastDriveTrain;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getRawAxis(1)) > Constants.ADVANCED_ARCADE_DRIVE_LINEAR_SPEED_DEAD_ZONE || Math.abs(joystick.getRawAxis(2)) > Constants.ADVANCED_ARCADE_DRIVE_ANGULAR_SPEED_DEAD_ZONE) {
      westCoastDriveTrain.advancedArcadeDrive(joystick.getRawAxis(1), joystick.getRawAxis(2), false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    westCoastDriveTrain.stopArcadeDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
