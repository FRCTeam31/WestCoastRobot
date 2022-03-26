// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class to manage the robot's limelight subsystem
 */
public class LimelightVisionSubsystem extends SubsystemBase {
  private NetworkTable limelightTable;
  private LinearFilter xFilter;
  private LinearFilter yFilter;
  private Double filteredX;
  private Double filteredY;
  private Double prevX, prevY, prevX2, prevY2;
  private int ticksWithNoTarget;
  private int maxTicksWithNoTarget;

  /** Creates a new LimelightVisionSubsystem.
   * @param maxTicskWithNoTarget The maximum number of ticks that the 
   * limelight will continue to interpolate values if no target is found
   */
  public LimelightVisionSubsystem(int maxTicksWithNoTarget) {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    xFilter = LinearFilter.singlePoleIIR(0.075, 0.02);
    yFilter = LinearFilter.singlePoleIIR(0.075, 0.02);
    this.maxTicksWithNoTarget = maxTicksWithNoTarget;
    prevX = 0.0;
    prevX2 = 0.0;
    prevY = 0.0;
    prevY2 = 0.0;

  }

  /**
   * 
   * @return The X-value of the limelight's current aquired target
   */
  public double getX(){
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  /**
   * 
   * @return The Y-value of the limelight's current aquired target
   */
  public double getY(){
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  /**
   * 
   * @return Whether or not there is a target (1 == valid target, 0 == no valid target)
   */
  public double getV(){
    return limelightTable.getEntry("tv").getDouble(0.0);
  }

  /**
   * 
   * @return The target's x-value
   */
  public double getFilteredX(){
    return filteredX;
  }

  /**
   * Return the filtered Y value. If there is no current target, return the most recent Y value
   * @return
   */
  public double getFilteredY(){
    return filteredY;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getV() != 0.0){
      // There is a valid target
      filteredX = xFilter.calculate(getX());
      filteredY = yFilter.calculate(getY());
      ticksWithNoTarget = 0;
    }
    else{
      // No Valid Target
      ticksWithNoTarget ++;
      if(ticksWithNoTarget > maxTicksWithNoTarget){
        // Do not interpolate anymore
        // Behave so that the target is at the center of the camera, and the Y value does not change
        filteredX = xFilter.calculate(0);
      }
      else{
        // Continue to interpolate
        filteredX = xFilter.calculate(prevX + (prevX - prevX2));
        filteredY = yFilter.calculate(prevY + (prevY - prevY2));
      }
    }
    // Update values
    prevX2 = prevX;
    prevY2 = prevY;
    prevX = filteredX;
    prevY = filteredY;

    // Display information
    SmartDashboard.putNumber("Limelight X", filteredX);
    SmartDashboard.putNumber("Limelight Y", filteredY);
    SmartDashboard.putNumber("Limelight V", getV());
  }
}
