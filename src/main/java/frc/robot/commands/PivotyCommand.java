// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;  

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.PivotyState;
import frc.robot.subsystems.PivotySubsystem;

public class PivotyCommand extends CommandBase {
  /** Creates a new PivotyCommand. */
  PivotySubsystem pivoty;
  double pivotySpeed;
  DigitalInput breakBeamOne;
  DigitalInput breakBeamTwo;
  int desiredEncoderValue;
  PivotyState pivotyState;
  boolean finish;
  Timer timer;
  //needs fixed
  public PivotyCommand(PivotySubsystem pivoty, int desiredEncoderValue, PivotyState pivotyState, boolean finish) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivoty = pivoty;
    this.desiredEncoderValue = desiredEncoderValue;
    this.pivotyState = pivotyState;
    this.finish = finish;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotyState.setEncoderValue(desiredEncoderValue);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotyState.setEncoderValue(desiredEncoderValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!finish){
    pivotyState.setEncoderValue(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(pivoty.getEncoderValue())>=Math.abs(desiredEncoderValue);
    if( finish){
      return pivoty.getEncoderValue() <= desiredEncoderValue+1000 && pivoty.getEncoderValue() >= desiredEncoderValue-1000 && timer.hasElapsed(2);
    }
    else{
    return false;
   }
  }
}
