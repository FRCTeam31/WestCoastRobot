// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PursellJaques.ShooterSpeeds;

/**
 * A subsystem to manage the robot's shooter.
 * This implementation uses two Falcon500 motors
 */
public class ShooterSubsystem extends SubsystemBase {

  // Instance Variables
  private WPI_TalonFX topMotor, bottomMotor;
  private ShooterSpeeds targetShooterSpeeds;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(WPI_TalonFX topMotor, WPI_TalonFX bottomMotor) {
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    this.targetShooterSpeeds = new ShooterSpeeds(0, 0);
  }

  /**
   * Get the current speed of the motors
   * 
   * @return an object describing the speeds of the top and bottom motors
   */
  public ShooterSpeeds getShooterSpeeds() {
    return new ShooterSpeeds(topMotor.getSelectedSensorPosition(), bottomMotor.getSelectedSensorPosition());
  }

  /**
   * Set the shooter speeds using the Falcon's simple PID controller
   * 
   * @param shooterSpeeds An object describing the shooter motor's target speeds
   */
  public void simpleSetShooterSpeeds(ShooterSpeeds shooterSpeeds) {
    topMotor.set(ControlMode.Velocity, shooterSpeeds.topMotorSpeed);
    bottomMotor.set(ControlMode.Velocity, -shooterSpeeds.bottomMotorSpeed);
    targetShooterSpeeds = shooterSpeeds;
    System.out.println(shooterSpeeds.bottomMotorSpeed);
  }
  
  /**
   * Stops the Shooter Motors
   */
  public void stopMotion() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
    targetShooterSpeeds = new ShooterSpeeds(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display Current Info
    SmartDashboard.putNumber("Top Shooter Motor Speed", getShooterSpeeds().topMotorSpeed);
    SmartDashboard.putNumber("Bottom Shooter Motor Speed", getShooterSpeeds().bottomMotorSpeed);
    SmartDashboard.putNumber("Top Shooter Motor Target Speed", targetShooterSpeeds.topMotorSpeed);
    SmartDashboard.putNumber("Bottom Shooter motor Target Speed", targetShooterSpeeds.bottomMotorSpeed);
  }
}
