// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem to controll the robot's intake. The intake should be powered by a
 * motor
 * connected to a TalonSRX motor controller
 */
public class IntakeSubsystem extends SubsystemBase {
  // Instance Variables
  private WPI_TalonSRX intakeMotor;
  private double currentPower;

  /**
   * Creates a new IntakeSubsystem.
   * 
   * @param intakeMotor the motor that will drive the intake
   */
  public IntakeSubsystem(WPI_TalonSRX intakeMotor) {
    this.intakeMotor = intakeMotor;
    currentPower = 0;
  }

  /**
   * Power the intake motor
   * 
   * @param power the power to give the motor, in the range [-1, 1]
   */
  public void driveIntake(double power) {
    currentPower = power;
    intakeMotor.set(power);
  }

  /**
   * Stop the motion of the intake motor
   */
  public void stopIntake(){
    intakeMotor.stopMotor();
    currentPower = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display information about the subsytem
    SmartDashboard.putNumber("Intake Motor Power", currentPower);
  }
}
