// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * A class to manage the Pixy subsystem.
 * This implementation uses an SPI connection to the pixy
 */
public class PixyVisionSubsystem extends SubsystemBase {
  // Instance Variables
  private Pixy2 pixy; 
  private Block largestBlock;
  private LinearFilter xFilter;
  private LinearFilter yFilter;
  private Double filteredX;
  private Double filteredY;
  private Double prevX, prevY, prevX2, prevY2;
  private int signature;
  private int ticksWithNoTarget;
  private int maxTicksWithNoTarget;

  /** Creates a new PixyVisionSubsystem. 
   * @param maxTicksWithNoTarget The maximum number of ticks without a target that the pixy will interpolate
   * @param signature The signature of the target
  */
  public PixyVisionSubsystem(int maxTicksWithNoTarget, int signature) {
    pixy = Pixy2.createInstance(new SPILink());
    System.out.println("PIXY CODE: " + pixy.init(5));
    this.maxTicksWithNoTarget = maxTicksWithNoTarget;
    this.signature = signature;
    xFilter = LinearFilter.singlePoleIIR(0.075, 0.02);
    yFilter = LinearFilter.singlePoleIIR(0.075, 0.02);
    prevX = 0.0;
    prevX2 = 0.0;
    prevY = 0.0;
    prevY2 = 0.0;
  }

  /**
     * 
     * @return the largets block from the pixy2 camera current image, null if none are found
     */
    public Block getLargestBlock(int signature){
      int blockCount = pixy.getCCC().getBlocks(false, signature, 25);

      if(blockCount <= 0){
          // No blocks found
          return null;
      }
      ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Get all the blocks from the pixy
      
      // Find block with largest area
      Block largestBlock = null;
      for(Block block: blocks){
          if(largestBlock == null){
              largestBlock = block;
          }
          if(block.getWidth() * block.getHeight() > largestBlock.getWidth() * largestBlock.getHeight()){
              largestBlock = block;
          }
      }
      this.largestBlock = largestBlock;
      return largestBlock;
  }

  /**
   * Change the signature to what is inputed 
   * @param signature The new signature to use
   */
  public void setSignature(int signature){
    this.signature = signature;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    largestBlock = getLargestBlock(signature);
    if(largestBlock != null){
      // There is a valid target
      filteredX = xFilter.calculate(largestBlock.getX());
      filteredY = yFilter.calculate(largestBlock.getY());
      ticksWithNoTarget = 0;
    }
    else{
      // No Valid Target
      ticksWithNoTarget ++;
      if(ticksWithNoTarget > maxTicksWithNoTarget){
        // Do not interpolate anymore
        // Behave so that the system knows there is no longer a valid target
        filteredX = null;
        filteredY = null;
      }
      else{
        // Continue to interpolate
        filteredX = xFilter.calculate(prevX + (prevX - prevX2));
        filteredY = yFilter.calculate(prevY + (prevY - prevY2));
      }
    }
    prevX2 = prevX;
    prevY2 = prevY;
    prevX = filteredX;
    prevY = filteredY;
    
    // Display Information

    if(largestBlock == null){
      SmartDashboard.putNumber("Pixy X", 99999);
      SmartDashboard.putNumber("Pixy Y", 99999);
    }
    else {
      SmartDashboard.putNumber("Pixy X", filteredX);
      SmartDashboard.putNumber("Pixy Y", filteredY);
    }
  }

  /**
   * @return The filtered X value from the pixy
   */
  public Double getFilteredX(){
    return filteredX;
  }
}
