// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TrackTargetWithTurretCommand extends CommandBase {
  // Instance Variables
  private LimelightVisionSubsystem limelightSubsystem;
  private TurretSubsystem turretSubsystem;

  /** Creates a new TrackTargetWithTurretCommand. 
   * @param turretSubsystem The turret that will be controlled
   * @param limelightSubsystem The limelight that the command will use to track the target
  */
  public TrackTargetWithTurretCommand(TurretSubsystem turretSubsystem, LimelightVisionSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsystem;

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.setTurretRelativeAngle(limelightSubsystem.getFilteredX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
