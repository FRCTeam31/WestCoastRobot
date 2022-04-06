// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
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
  public SimpleAuton(WestCoastDriveTrain driveTrain, TurretSubsystem turret, ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SimpleAutoDriveAndIntakeCommand(driveTrain, intakeSubsystem, 1, 1, false),
      new SimpleTurnRobot(driveTrain, 180, 3),
      new ShootBallAtSetSpeedCommand(shooter, intakeSubsystem, 2, 2, 0.3 * Constants.MAX_SHOOTER_SPEED)

    );
  }
}
