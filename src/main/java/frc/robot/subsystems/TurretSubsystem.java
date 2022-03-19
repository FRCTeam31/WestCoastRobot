// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Properties;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A subsystem to control the robots turret (different from the shooter). 
 * This implementation uses a Falcon500 motor
 */
public class TurretSubsystem extends SubsystemBase {
  // Instance Variables
  private WPI_TalonFX turretMotor;
  private double turretAngleAlignmentConstant; // Negative the turret motor's sensor position when the turret is at its zero-degree position
  private Double maxTurretAngle;
  private double targetAngle;


  /** Creates a new TurretSubsystem. 
   * @param turretMotor The motor that will control the turret
   * @param alignmentConstants The properties object to load from - Turret will use the "TURRET" property
  */
  public TurretSubsystem(WPI_TalonFX turretMotor, Properties alignmentConstants) {
    this.turretMotor = turretMotor;
    this.loadAngleAlignmentConstant(alignmentConstants);
    maxTurretAngle = null;
    targetAngle = this.getCurrentAngle();
  }

  /** Creates a new TurretSubsystem, with no modified zero-point
   * @param turretMotor The motor that will control the turret
  */
  public TurretSubsystem(WPI_TalonFX turretMotor) {
    this.turretMotor = turretMotor;
    turretAngleAlignmentConstant = 0;
    maxTurretAngle = null;
    targetAngle = this.getCurrentAngle();
  }

  /**
   * Load the turret's turretAngleAlignmentConstant (the constant that sets the turret's zero-degree point)
   * @param alignmentConstants The properties object to load from - Turret will use the "TURRET" property
   */
  public void loadAngleAlignmentConstant(Properties alignmentConstants){
    turretAngleAlignmentConstant = Double.parseDouble(alignmentConstants.getProperty("TURRET", "0.0"));
    System.out.println("Turret Alignment Constant set to: " + turretAngleAlignmentConstant);
  }

  /**
   * Set the maximum allowable rotation of the turret (relative to the zero-point)
   * @param maxTurretAngle Max allowable rotation in degrees
   */
  public void setMaxTurretAngle(double maxTurretAngle){
    this.maxTurretAngle = maxTurretAngle;
  }

  /**
   * @return The turret's current raw position in motor ticks
   */
  public double getCurrentRawSensorPosition(){
    return turretMotor.getSelectedSensorPosition();
  }

  /**
   * 
   * @return The current sensor position in ticks, modified so that it is relative to the set zero-point
   */
  public double getCurrentSensorPosition(){
    return turretMotor.getSelectedSensorPosition() + turretAngleAlignmentConstant;
  }

  /**
   * 
   * @return The turret's current angle, relative to the preset zero-point
   */
  public double getCurrentAngle(){
    return this.getCurrentSensorPosition() * Constants.TURRET_TICKS_TO_DEGREES_CONSTANT;
  }

  /**
   * Set the shooter to an angle relative to the zero point.
   * If a maxTurretAngle has been set, the turret will not spin past that point
   * @param angle The angle, in degrees, that the turret should turn to
   */
  public void setTurretAbsoluteAngle(double angle){
    if(maxTurretAngle != null){
      // If angle is outside of bounds, clamp it
      if(Math.abs(angle) > maxTurretAngle){
        angle = Math.signum(angle) * maxTurretAngle;
      }
    }

    this.targetAngle = angle;
    double targetTicks = angle * Constants.TURRET_DEGREES_TO_TICKS_CONSTANT + turretAngleAlignmentConstant;
    turretMotor.set(ControlMode.Position, targetTicks);
  }

  /**
   * Set the turret to an angle relative to the current position. 
   * If a maxTurretAngle has been set, the turret will not spin past that point
   * @param angle The angle, in degrees, that the turret should turn to
   */
  public void setTurretRelativeAngle(double angle){
    this.setTurretAbsoluteAngle(this.getCurrentAngle() + angle);
  }

  /**
   * Stop the motion of the turret motor
   */
  public void stopMotion(){
    turretMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display current Info
    SmartDashboard.putNumber("Turret Angle", this.getCurrentAngle());
    SmartDashboard.putNumber("Target Turret Angle", targetAngle);
  }
}
