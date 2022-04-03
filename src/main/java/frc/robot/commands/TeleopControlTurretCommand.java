// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TeleopControlTurretCommand extends CommandBase {
  // Instance Variables
  private TurretSubsystem turret;
  private Joystick joystick;
  private int axis;

  /** Creates a new TeleopControlTurretCommand. 
   * @param turretSubsystem The turret that the command will control
   * @param joystick The joystick that will control this motion
   * @param axis The joystick axis that will control the motion
  */
  public TeleopControlTurretCommand(TurretSubsystem turretSubsystem, Joystick joystick, int axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turretSubsystem;
    this.joystick = joystick;
    this.axis = axis;

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(joystick.getRawAxis(axis)) > 0.1){
      turret.setTurretRelativeAngle(joystick.getRawAxis(axis) * 40);
    }
    else{
      turret.stopMotion();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
