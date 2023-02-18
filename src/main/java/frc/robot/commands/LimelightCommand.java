// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.LimelightState;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends CommandBase {
  LimelightState limelightState;
  private LimelightSubsystem limelight;
  private int pipeline;
  public boolean limelightOn;
  
  
  /** Creates a new LimelightCommand. */
  public LimelightCommand(LimelightSubsystem limelight, int pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.pipeline = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightState.limelightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.setPipeline(pipeline);
    limelight.updateIsThereTarget();
    limelight.updateXOffset();
    limelight.whatToDo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    limelightState.limelightOff();
    return false;
  }
}