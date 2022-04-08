// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.PursellJaques.AutoUtill;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;
import io.github.pseudoresonance.pixy2api.Pixy2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new SimpleThreeBallAuto. */
  public SimpleThreeBallAuto(WestCoastDriveTrain driveTrain, TurretSubsystem turret, ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, LimelightVisionSubsystem limelight, PixyVisionSubsystem pixy, AutoSetShooterSpeed autoShootSpeed) {
    
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new SimpleAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 2, 2, false),
      // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 2, 3).andThen(() -> driveTrain.driveWithVoltage(0, 0)),
      // new TurnToAngleWithPIDCommand(driveTrain, 180, 3),
      // new AutoPowerIntake(intakeSubsystem, 0.7, -Constants.SAFE_INTAKE_POWER),
      // new WaitCommand(0.1),
      // new ShootBallAtSetSpeedCommand(shooter, intakeSubsystem, 0.2, 0, 0.7 * Constants.MAX_SHOOTER_SPEED), // Spin shooter up 
      // new ParallelRaceGroup(
      //   // new ShootBallAtSetSpeedCommand(shooter, intakeSubsystem, 5, 5, 0.42 * Constants.MAX_SHOOTER_SPEED), 
      //   autoShootSpeed,
      //   new AutoPowerIntake(intakeSubsystem, 3, 1),
      //   new TrackTargetWithTurretCommand(turret, limelight)),
      
      // new TurnToAngleWithPIDCommand(driveTrain, -180, 3).andThen(() -> driveTrain.resetPos()),
      // AutoUtill.getRamseteCommandToCoordinate(driveTrain, 2.29, 0.58, new Rotation2d()) // Find Third ball
      // // new TrackBallWithPixyCommand(driveTrain, pixy, 2), // Track it / get in front of it
      // // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 0.5, 5).andThen(() -> driveTrain.driveWithVoltage(0, 0)),// Intake ball
      // // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, -1, 0),
      // // new TurnToAngleWithPIDCommand(driveTrain, 180, 2), // Turn to face goal
      // // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 1.5, 0).andThen(() -> driveTrain.driveWithVoltage(0, 0)), // Drive to be closr to goal,

      // // // Shoot last ball
      // // new AutoPowerIntake(intakeSubsystem, 0.7, -Constants.SAFE_INTAKE_POWER),
      // // new WaitCommand(0.1),
      // // new ShootBallAtSetSpeedCommand(shooter, intakeSubsystem, 0.2, 0, 0.42 * Constants.MAX_SHOOTER_SPEED), // Spin shooter up
      // // new ParallelCommandGroup(
      // //   // new ShootBallAtSetSpeedCommand(shooter, intakeSubsystem, 5, 5, 0.42 * Constants.MAX_SHOOTER_SPEED), 
      // //   autoShootSpeed,
      // //   new AutoPowerIntake(intakeSubsystem, 5, 1),
      // //   new TrackTargetWithTurretCommand(turret, limelight))

    // NEW SECTION_____________!!!!!!
    AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 2, 3).andThen(() -> driveTrain.driveWithVoltage(0, 0)),
    new ParallelCommandGroup(
      new TurnToAngleWithPIDCommand(driveTrain, 180, 2),
      new AutoPowerIntake(intakeSubsystem, 0.5, -0.8),
      new ParallelRaceGroup(
        autoShootSpeed,
        new WaitCommand(1)
      )
    )

    // new ParallelRaceGroup(
    //   autoShootSpeed,
    //   new TrackTargetWithTurretCommand(turret, limelight),
    //   new SequentialCommandGroup(
    //     new WaitCommand(0.1),
    //     new AutoPowerIntake(intakeSubsystem, 3, 1)
    //   )
    // ),

    // new TurnToAngleWithPIDCommand(driveTrain, -180, 2),
    // AutoUtill.getRamseteCommandToCoordinate(driveTrain, 2.29, 0.58, new Rotation2d()), // Find Third ball
    // new TrackBallWithPixyCommand(driveTrain, pixy, 2),
    // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 0.4, 3),
    // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, -1, 0),
    // new TurnToAngleWithPIDCommand(driveTrain, 180, 2),
    // AutoUtill.getAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 2, 0),
    // new ParallelCommandGroup(
    //   autoShootSpeed,
    //   new TrackTargetWithTurretCommand(turret, limelight),
    //   new SequentialCommandGroup(
    //     new AutoPowerIntake(intakeSubsystem, 0.4, -0.5),
    //     new WaitCommand(0.1),
    //     new AutoPowerIntake(intakeSubsystem, 10, 1)
    //   )
    // )
    

    );
  }
}
