// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PursellJaques;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoPowerIntake;
import frc.robot.commands.AutoSetShooterSpeed;
import frc.robot.commands.TrackTargetWithOdometryCommand;
import frc.robot.commands.TrackTargetWithRobotUsingOdometryCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;

/** Add your docs here.
 * A class with utilities for autonomous features / commands
 */
public class AutoUtill {
    /**
     * Return a parallel command group that includes a RamseteCommand that tells the robot to follow a
     * directly forward trajectory for a specified distance and a PowerIntake command that powers the intake for a specified time
     * @param driveTrain The drivetrain that the command will use
     * @param distance The distance in meters that the robot will travel
     * @param intakeTime The time that the intake should be powered for
     * @return The generated parallel command group
     */
    public static ParallelCommandGroup getAutoDriveAndIntakeCommand(WestCoastDriveTrain driveTrain, IntakeSubsystem intake, double distance, double intakeTime){
        // Calculate target pos (forward the distance in the same direction the robot is facing)
        Pose2d initialPose2d = driveTrain.getPose();
        double deltaX = distance * Math.sin(-initialPose2d.getRotation().getRadians());
        double deltaY = distance * Math.cos(-initialPose2d.getRotation().getRadians());

        Pose2d targetPose2d = new Pose2d(
            new Translation2d(initialPose2d.getX() + deltaX, initialPose2d.getY() + deltaY), 
            initialPose2d.getRotation());
        
        // Create RamseteCommand
        TrajectoryConfig config = new TrajectoryConfig(Constants.TRAJECTORY_MAX_SPEED, Constants.TRAJECTORY_MAX_ACCELERATION);
        config.setKinematics(driveTrain.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            initialPose2d, 
            List.of(), 
            targetPose2d, 
            config);

        SequentialCommandGroup ramseteCommand = new RamseteCommand(
            trajectory, 
            driveTrain::getPose, 
            new RamseteController(), 
            driveTrain.getFeedForward(), 
            driveTrain.getKinematics(), 
            driveTrain::getWheelSpeeds, 
            driveTrain.getLeftPIDController(), 
            driveTrain.getRightPIDController(), 
            driveTrain::driveWithVoltage,
            driveTrain).andThen(() -> driveTrain.stopArcadeDrive());
        
        // Return Command
        return new ParallelCommandGroup(
            ramseteCommand,
            new AutoPowerIntake(intake, intakeTime)
        );
    }

    /**
     * Get a parallel command groupo that turns the robot to face the target, 
     * turns the turret to face the target, and then powers the intake and 
     * power the shooter motors for a set time
     * @param driveTrain The drive train the command will use
     * @param turret The turret the command will use
     * @param shooter The shooter the command will use
     * @param intake The intake that the command will use
     * @param limelight The limelight that the command will use
     * @param maxTime The maximum time that the that the command should run in seconds
     * @param intakeWaittime The time that the intake should wait to be powered in seconds
     * @param intakePowerTime The time that the intake should be powered after waiting in seconds
     * @return
     */
    public static Command getAutoShootCommand(WestCoastDriveTrain driveTrain, TurretSubsystem turret, ShooterSubsystem shooter, IntakeSubsystem intake, LimelightVisionSubsystem limelight, double maxTime, double intakeWaittime, double intakePowerTime){
        TrackTargetWithOdometryCommand trackTargetWithOdometryCommand = new TrackTargetWithOdometryCommand(turret, limelight, driveTrain, Constants.TRACK_TARGET_WITH_ODOMETRY_LIMELIGHT_ZONE);
        TrackTargetWithRobotUsingOdometryCommand trackTargetWithRobotUsingOdometryCommand = new TrackTargetWithRobotUsingOdometryCommand(driveTrain, maxTime);
        AutoSetShooterSpeed autoSetShooterSpeed = new AutoSetShooterSpeed(shooter, limelight, Constants.AUTO_SHOOTER_SPEED_RANGES, Constants.AUTO_SHOOTER_SPEED_TOP_MOTOR_SPEEDS, Constants.AUTO_SHOTER_SPEED_BOTTOM_MOTOR_SPEEDS);

        return new ParallelRaceGroup(
            trackTargetWithRobotUsingOdometryCommand,
            trackTargetWithOdometryCommand,
            autoSetShooterSpeed,
            new SequentialCommandGroup(
                new WaitCommand(intakeWaittime),
                new AutoPowerIntake(intake, intakePowerTime)
            )

        );
    }
}
