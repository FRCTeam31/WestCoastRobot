// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
  // Instance Variables
  private WPI_TalonFX climbingMotor;

  /** Creates a new ClimbingSubsystem.
   * @param climbingMotor The motor that will be used for climbing
   */
  public ClimbingSubsystem(WPI_TalonFX climbingMotor) {
    this.climbingMotor = climbingMotor;
  }

  /**
   * Power the motor
   * @param power The power to give the motor, [-1. 1]. 1 is extend
   */
  public void powerClimber(double power){
    // min: -1613166.000000
    // max: 1159798.000000
    double currentPose = climbingMotor.getSelectedSensorPosition();

    if(currentPose > -1159798.000000){
      // Climbing is at its max
      if(power < 0){
        climbingMotor.set(power);
      }
      else{
        climbingMotor.set(0);
      }
    }
    else if(currentPose < -1713166.000000){
      // Climbing is at its min
      if(power > 0){
        climbingMotor.set(power);
      }
      else{
        climbingMotor.set(0);
      }
    }
    else{
      climbingMotor.set(power);
    }

    
    System.out.println(power);
  }

  /**
   * Stop the climbing motor
   */
  public void stopMotor(){
    climbingMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Pos", climbingMotor.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }
}
