// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeSubsystem;

public class intakeCommand extends CommandBase {
  /** Creates a new intakeCommand. */
  intakeSubsystem intake;
  double openSpeed;
  double closedSpeed;
  public intakeCommand(intakeSubsystem intake, double openSpeed, double closedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.openSpeed = openSpeed;
    this.closedSpeed = closedSpeed;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeClose(closedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOpen(openSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
