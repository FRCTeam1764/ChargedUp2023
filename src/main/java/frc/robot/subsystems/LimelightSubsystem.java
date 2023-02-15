// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private static NetworkTable table;
  private double xOffset;
  private double isThereTarget;
  
  public LimelightSubsystem(NetworkTable table) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    isThereTarget = table.getEntry("tv").getDouble(0);
    xOffset = table.getEntry("tx").getDouble(0);
    
  }

  public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getIsThereTarget() {
    return isThereTarget;
  }





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
