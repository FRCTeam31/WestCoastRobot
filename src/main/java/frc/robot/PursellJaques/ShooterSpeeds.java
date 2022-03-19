// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PursellJaques;

/**
 * A wrapper class for the shooter speeds, holding both the values
 */
public class ShooterSpeeds {
    public double topMotorSpeed, bottomMotorSpeed;

    /**
     * 
     * @param topMotorSpeed Speed of the top motor in ticks
     * @param bottomMotorSpeed Speed of the bottom motr in ticks 
     */
    public ShooterSpeeds(double topMotorSpeed, double bottomMotorSpeed){
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed;
    }
}
