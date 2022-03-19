// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class TeleopControlIntakeCommand extends CommandBase {
  // Instance Variables
  private IntakeSubsystem intakeSubsystem;
  private Joystick joystick;
  private int axis;

  /** Creates a new TeleopControlIntakeCommand.
   * @param intakeSubsystem The intake that the command will operate
   */
  public TeleopControlIntakeCommand(IntakeSubsystem intakeSubsystem, Joystick joystick, int axis) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.joystick = joystick;
    this.axis = axis;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.driveIntake(joystick.getRawAxis(axis));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}