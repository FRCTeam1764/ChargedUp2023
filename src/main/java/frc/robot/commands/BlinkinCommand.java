// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;

public class BlinkinCommand extends CommandBase {
  /** Creates a new BlinkinCommand. */
  double LEDColor;
BlinkinSubsystem Blinkin;
  public BlinkinCommand(double LEDColor) {
this.LEDColor = LEDColor;
this.Blinkin = new BlinkinSubsystem();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Blinkin.setLEDs(LEDColor);
    System.out.println("It gets here");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
