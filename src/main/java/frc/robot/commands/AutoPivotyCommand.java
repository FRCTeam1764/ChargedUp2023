// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.PivotyState;
import frc.robot.subsystems.PivotySubsystem;

public class AutoPivotyCommand extends CommandBase {
  /** Creates a new AutoPivotyCommand. */
  PivotySubsystem pivoty;
  double pivotySpeed;
  double desiredEncoderValue;
  PivotyState pivotyState;
  public AutoPivotyCommand(PivotySubsystem pivoty, double desiredEncoderValue, PivotyState pivotyState) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.pivoty = pivoty;
      this.desiredEncoderValue = desiredEncoderValue;
      this.pivotyState = pivotyState;

      // this.sideRollers = sideRollers;
      // this.backRollers = backRollers;
     
      // addRequirements(sideRollers,backRollers);
    }  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotyState.setEncoderValue(desiredEncoderValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(
    
  ) { pivotyState.setEncoderValue(desiredEncoderValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivoty.getEncoderValue() <= desiredEncoderValue+4000 && pivoty.getEncoderValue() >= desiredEncoderValue-4000;
  }
}
