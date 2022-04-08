// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.PursellJaques.AutoUtill;
import frc.robot.PursellJaques.ShooterSpeeds;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * This is intended to drive strait to pickup second ball and taxi, then
 * turn and shoot both
 */
public class SimpleAuton extends SequentialCommandGroup {
  /** Creates a new SimpleAuton. */
  public SimpleAuton(WestCoastDriveTrain driveTrain, TurretSubsystem turret, ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, LimelightVisionSubsystem limelight, AutoSetShooterSpeed autoShootSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new SimpleAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 2, 2, false),
      AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 2, 3).andThen(() -> driveTrain.driveWithVoltage(0, 0)),
      new TurnToAngleWithPIDCommand(driveTrain, 180, 3),
      new AutoPowerIntake(intakeSubsystem, 0.7, -Constants.SAFE_INTAKE_POWER),
      new WaitCommand(0.1),
      new ParallelCommandGroup(
        // new ShootBallAtSetSpeedCommand(shooter, intakeSubsystem, 5, 5, 0.42 * Constants.MAX_SHOOTER_SPEED), 
        autoShootSpeed,
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new AutoPowerIntake(intakeSubsystem, 5, 1)),
        new TrackTargetWithTurretCommand(turret, limelight)).andThen(() -> shooter.simpleSetShooterSpeeds(new ShooterSpeeds(0, 0))
        )
      

    );
  }
}
