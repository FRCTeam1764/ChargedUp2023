// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;

public class BlinkinCommand extends CommandBase {
  /** Creates a new BlinkinCommand. */
BlinkinSubsystem Blinkin;
//needs toggle and state
  public BlinkinCommand( BlinkinSubsystem Blinkin) {
  this.Blinkin = Blinkin;
  addRequirements(Blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Blinkin.setLEDs(.67);
    System.out.println("hello world");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Blinkin.setLEDs(.89);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
