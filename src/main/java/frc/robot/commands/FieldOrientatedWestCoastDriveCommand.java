// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WestCoastDriveTrain;

public class FieldOrientatedWestCoastDriveCommand extends CommandBase {
  // Instance Variables
  private WestCoastDriveTrain driveTrain;
  private Joystick joystick;

  /** Creates a new FieldOrientatedWestCoastDriveCommand. 
   * @param driveTrain The drive train that the command will control
   * @param joystick The joystick that will control the driving
  */
  public FieldOrientatedWestCoastDriveCommand(WestCoastDriveTrain driveTrain, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.joystick = joystick;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.instantiateFieldOrientatedDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.fieldOreintatedNoah(joystick.getMagnitude(), joystick.getDirectionDegrees());
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
