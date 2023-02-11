// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;  

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotySubsystem;

public class PivotyCommand extends CommandBase {
  /** Creates a new PivotyCommand. */
  PivotySubsystem pivoty;
  double pivotySpeed;
  DigitalInput breakBeamOne;
  DigitalInput breakBeamTwo;
  //needs fixed
  public PivotyCommand(PivotySubsystem pivoty, double pivotySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivoty = pivoty;
    this.pivotySpeed = pivotySpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoty.pivotyOn(pivotySpeed, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivoty.pivotyOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
