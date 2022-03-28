// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WestCoastDriveTrain;

/**
 * A command that allowes simple driving with the WestCoastDriveTrain subsystem
 */
public class SimpleWestCoastDriveCommand extends CommandBase {
  // Instance Variables
  private WestCoastDriveTrain westCoastDrive;
  private Joystick joystick;
  private boolean squareInputs;

  /** Creates a new SimpleWestCoastDriveCommand.
   * @param westCoastDrive The west coast drive system
   * @param joystick the joystick that will control the driving
   * @param squareInputs whether or not to square inputs
   */
  public SimpleWestCoastDriveCommand(WestCoastDriveTrain westCoastDrive, Joystick joystick, boolean squareInputs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(westCoastDrive);

    this.westCoastDrive = westCoastDrive;
    this.joystick = joystick;
    this.squareInputs = squareInputs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    westCoastDrive.simpleArcadeDrive(joystick.getRawAxis(5), joystick.getRawAxis(4), squareInputs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    westCoastDrive.stopArcadeDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
