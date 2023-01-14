// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotySubsystem;

public class PivotyCommand extends CommandBase {
  /** Creates a new PivotyCommand. */
  PivotySubsystem pivotySubsystem;
  double pivotySpeed;
  public PivotyCommand(double pivotySpeed, PivotySubsystem pivotySubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotySpeed = pivotySpeed;
    this.pivotySubsystem = pivotySubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotySubsystem.PivotyOn(pivotySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotySubsystem.PivotyOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //"needs probably encorder or something" - Zach
  }
}
