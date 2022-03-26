// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PursellJaques.InterpolatingTreeMap;
import frc.robot.PursellJaques.ShooterSpeeds;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A class for automated shooting speed setting
 */
public class AutoSetShooterSpeed extends CommandBase {
  // Instance Variables
  private InterpolatingTreeMap topMotorMap;
  private InterpolatingTreeMap bottomMotorMap;
  private ShooterSubsystem shooterSubsystem;
  private LimelightVisionSubsystem limelight;

  /** Creates a new AutoSetShooterSpeed. 
   * @param shooterSubsystem The shooter subsystem that the command will control
   * @param limelihght The limelight that the command will use 
   * @param ranges The list of ranges / distances / value that acts as a proxy for distance. Make sure that the index of each distance corresponds with the index of the corresponding shooter speed.
   * @param topMotorSpeeds The list of speeds of the top motor that correspond with the ranges
   * @param bottomMotorSpeeds The list of speeds of the bottom motor that correspond with the ranges
   */
  public AutoSetShooterSpeed(ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem limelight, double[] ranges, double[] topMotorSpeeds, double[] bottomMotorSpeeds) {
    // Fill up tree maps
    topMotorMap = new InterpolatingTreeMap();
    bottomMotorMap = new InterpolatingTreeMap();

    for(int index = 0; index < ranges.length; index ++){
      topMotorMap.put(ranges[index], topMotorSpeeds[index]);
      bottomMotorMap.put(ranges[index], bottomMotorSpeeds[index]);
    }
    this.limelight = limelight;
    this.shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the shooter speeds to 0
    shooterSubsystem.simpleSetShooterSpeeds(new ShooterSpeeds(0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double ty = limelight.getFilteredY();

    shooterSubsystem.simpleSetShooterSpeeds(new ShooterSpeeds(
      topMotorMap.getInterpolatedValue(ty), 
      bottomMotorMap.getInterpolatedValue(ty))
    );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.simpleSetShooterSpeeds(new ShooterSpeeds(0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
