// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

import java.io.FileInputStream;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FieldOrientatedWestCoastDriveCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.SimpleWestCoastDriveCommand;
import frc.robot.commands.TeleopControlIntakeCommand;
import frc.robot.commands.TeleopControlTurretCommand;
import frc.robot.commands.TrackTargetWithTurretCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.PixyVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WestCoastDriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.SerialPort;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Instance Variables

  // Alighnment Constants
  private Properties alignmentConstants;

  // User Input Devices
  private Joystick joystick0;
  private JoystickButton joystick0Button1;
  private JoystickButton joystick0Button2;

  // Sensors
  //NavX
  private AHRS navx;
  // Limelight
  private LimelightVisionSubsystem limelightVisionSubsystem;
  // Pixy 
  private PixyVisionSubsystem pixyVisionSubsystem;

  // Drive Train
  // Drive Train Hardware
  private WPI_TalonFX leftDriveMotor1, leftDriveMotor2, leftDriveMotor3, 
                      rightDriveMotor1, rightDriveMotor2, rightDriveMotor3;
  private WestCoastDriveTrain westCoastDriveTrain;
  // Drive Train Commands
  private SimpleWestCoastDriveCommand simpleWestCoastDriveCommand;
  private FieldOrientatedWestCoastDriveCommand fieldOrientatedWestCoastDriveCommand;

  // Intake Subsystem
  private IntakeSubsystem intakeSubsystem;
  private TeleopControlIntakeCommand teleopControlIntakeCommand;

  // Turret Subsystem
  private TurretSubsystem turretSubsystem;
  private TeleopControlTurretCommand teleopControlTurretCommand;

  // Shooter Subsystem
  private ShooterSubsystem shooterSubsystem;
  private SetShooterSpeedCommand setShooterSpeedCommand;

  // Automation Commands
  private TrackTargetWithTurretCommand trackTargetWithTurretCommand;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Alighnment Constants
    // try {
    //   alignmentConstants = new Properties();
    //   alignmentConstants.load(new FileInputStream(ALIGNMENT_CONSTANTS_FILE_PATH));
    //   System.out.println("---- Loading Alignment Constants SUCCEEDED ----");
    // } catch (Exception e) {
    //   e.printStackTrace();
    //   System.out.println("---- Loading Alignment Constants FAILED ----");
    // }

    // User Input Devices
    joystick0 = new Joystick(JOYSTICK_0_PORT_NUMBER);
    joystick0Button1 = new JoystickButton(joystick0, 1);
    joystick0Button2 = new JoystickButton(joystick0, 2);

    // Sensors
    // NavX
    navx = new AHRS(SerialPort.Port.kUSB);
    // Limelight Subsystem
    limelightVisionSubsystem = new LimelightVisionSubsystem(Constants.LIMELIGHT_MAX_TIME_WITH_NO_TARGET);
    // Pixy Subsystem
    pixyVisionSubsystem = new PixyVisionSubsystem(Constants.PIXY_MAX_TIME_WITH_NO_TARGET, Constants.CURRENT_BALL_SIGNATURE);

    // // Drive Train
    // // Drive Train Hardware
    // leftDriveMotor1 = new WPI_TalonFX(LEFT_DRIVE_MOTOR_1_CAN_ID);
    // leftDriveMotor2 = new WPI_TalonFX(LEFT_DRIVE_MOTOR_2_CAN_ID);
    // leftDriveMotor3 = new WPI_TalonFX(LEFT_DRIVE_MOTOR_3_CAN_ID);
    // rightDriveMotor1 = new WPI_TalonFX(RIGHT_DRIVE_MOTOR_1_CAN_ID);
    // rightDriveMotor2 = new WPI_TalonFX(RIGHT_DRIVE_MOTOR_2_CAN_ID);
    // rightDriveMotor3 = new WPI_TalonFX(RIGHT_DRIVE_MOTOR_3_CAN_ID);
    // TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    // driveMotorConfig.slot0.kP = DRIVE_MOTOR_KP;
    // driveMotorConfig.slot0.kI = DRIVE_MOTOR_KI;
    // driveMotorConfig.slot0.kD = DRIVE_MOTOR_KD;
    // driveMotorConfig.slot0.kF = DRIVE_MOTOR_KF;
    // leftDriveMotor1.configAllSettings(driveMotorConfig);
    // leftDriveMotor2.configAllSettings(driveMotorConfig);
    // leftDriveMotor3.configAllSettings(driveMotorConfig);
    // rightDriveMotor1.configAllSettings(driveMotorConfig);
    // rightDriveMotor2.configAllSettings(driveMotorConfig);
    // rightDriveMotor3.configAllSettings(driveMotorConfig);
    // leftDriveMotor1.selectProfileSlot(0, 0);
    // leftDriveMotor2.selectProfileSlot(0, 0);
    // leftDriveMotor3.selectProfileSlot(0, 0);
    // rightDriveMotor1.selectProfileSlot(0, 0);
    // rightDriveMotor2.selectProfileSlot(0, 0);
    // rightDriveMotor3.selectProfileSlot(0, 0);
    // westCoastDriveTrain = new WestCoastDriveTrain(
    //   new WPI_TalonFX[] {leftDriveMotor1, leftDriveMotor2, leftDriveMotor3}, 
    //   new WPI_TalonFX[] {rightDriveMotor1, rightDriveMotor2, rightDriveMotor3},
    //   navx,
    //   Constants.DRIVE_TRAIN_WIDTH_METERS);
    // // Drive Train Commands
    // simpleWestCoastDriveCommand = new SimpleWestCoastDriveCommand(westCoastDriveTrain, joystick0, SIMPLE_WEST_COAST_DRIVE_COMMAND_SQUARE_INPUTS);
    // westCoastDriveTrain.setDefaultCommand(simpleWestCoastDriveCommand);
    // fieldOrientatedWestCoastDriveCommand = new FieldOrientatedWestCoastDriveCommand(westCoastDriveTrain, joystick0);


    // // Intake Subsystem
    // WPI_TalonSRX intakeMotor = new WPI_TalonSRX(INTAKE_MOTOR_CAN_ID);
    // intakeSubsystem = new IntakeSubsystem(intakeMotor);
    // teleopControlIntakeCommand = new TeleopControlIntakeCommand(intakeSubsystem, joystick0, Constants.TELEOP_BALL_INTAKE_JOYSTICK_AXIS);
    // intakeSubsystem.setDefaultCommand(teleopControlIntakeCommand);

    // // Turret Subsystem
    // WPI_TalonFX turretMotor = new WPI_TalonFX(TURRET_MOTOR_CAN_ID);
    // TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
    // turretMotorConfig.slot0.kP = TURRET_MOTOR_KP;
    // turretMotorConfig.slot0.kI = TURRET_MOTOR_KI;
    // turretMotorConfig.slot0.kD = TURRET_MOTOR_KD;
    // turretMotorConfig.slot0.kF = TURRET_MOTOR_KF;
    // turretMotor.configAllSettings(turretMotorConfig);
    // turretMotor.selectProfileSlot(0, 0);
    // turretSubsystem = new TurretSubsystem(turretMotor);
    // turretSubsystem.setMaxTurretAngle(TURRET_MAX_ANGLE);
    // teleopControlTurretCommand = new TeleopControlTurretCommand(turretSubsystem, joystick0, Constants.TELEOP_CONTROL_TURRET_JOYSTICK_AXIS);
    // turretSubsystem.setDefaultCommand(teleopControlTurretCommand);

    // Shooter Subsystem
    // WPI_TalonFX topShooterMotor = new WPI_TalonFX(TOP_SHOOTER_MOTOR_CAN_ID);
    // WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(BOTTOM_SHOOTER_MOTOR_CAN_ID);
    // TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    // shooterMotorConfig.slot0.kP = SHOOTER_MOTOR_KP;
    // shooterMotorConfig.slot0.kI = SHOOTER_MOTOR_KI;
    // shooterMotorConfig.slot0.kD = SHOOTER_MOTOR_KD;
    // shooterMotorConfig.slot0.kF = SHOOTER_MOTOR_KF;
    // topShooterMotor.configAllSettings(shooterMotorConfig);
    // bottomShooterMotor.configAllSettings(shooterMotorConfig);
    // topShooterMotor.selectProfileSlot(0, 0);
    // bottomShooterMotor.selectProfileSlot(0, 0);
    // shooterSubsystem = new ShooterSubsystem(topShooterMotor, bottomShooterMotor);
    // setShooterSpeedCommand = new SetShooterSpeedCommand(Constants.SET_SHOOTER_SPEED_TOP_SHOOTER_MOTOR_AXIS, Constants.SET_SHOOTER_SPEED_Bottom_SHOOTER_MOTOR_AXIS, joystick0, shooterSubsystem);

    // // Automation Commands
    // trackTargetWithTurretCommand = new TrackTargetWithTurretCommand(turretSubsystem, limelightVisionSubsystem);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // joystick0Button1.toggleWhenPressed(fieldOrientatedWestCoastDriveCommand, true);
    // joystick0Button2.toggleWhenPressed(trackTargetWithTurretCommand, true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig config = new TrajectoryConfig(Constants.TRAJECTORY_MAX_SPEED, Constants.TRAJECTORY_MAX_ACCELERATION);
    config.setKinematics(westCoastDriveTrain.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(), 
      List.of(new Translation2d(1, 1)), 
      new Pose2d(), 
      config);
    
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      westCoastDriveTrain :: getPose, 
      new RamseteController(), 
      westCoastDriveTrain.getFeedForward(), 
      westCoastDriveTrain.getKinematics(), 
      westCoastDriveTrain::getWheelSpeeds, 
      westCoastDriveTrain.getLeftPIDController(), 
      westCoastDriveTrain.getRightPIDController(), 
      westCoastDriveTrain::driveWithVoltage, 
      westCoastDriveTrain);

    return ramseteCommand.andThen(() -> westCoastDriveTrain.driveWithVoltage(0, 0));
  }
}
  