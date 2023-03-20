// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;  

import javax.management.modelmbean.RequiredModelMBean;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.PivotyState;
import frc.robot.subsystems.BackRollers;
import frc.robot.subsystems.PivotySubsystem;
import frc.robot.subsystems.SideRollers;

public class PivotyCommand extends CommandBase {
  /** Creates a new PivotyCommand. */
  PivotySubsystem pivoty;
  double pivotySpeed;
  DigitalInput breakBeamOne;
  DigitalInput breakBeamTwo;
  int desiredEncoderValue;
  PivotyState pivotyState;
  BackRollers backRollers;
  SideRollers sideRollers;
  double speed;
  boolean finish;
  Timer timer;
  //needs fixed
  public PivotyCommand(PivotySubsystem pivoty, int desiredEncoderValue, PivotyState pivotyState, boolean finish,SideRollers sideRollers,BackRollers backRollers, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivoty = pivoty;
    this.desiredEncoderValue = desiredEncoderValue;
    this.pivotyState = pivotyState;
    this.finish = finish;
    // this.sideRollers = sideRollers;
    // this.backRollers = backRollers;
    this.speed = speed;
    // addRequirements(sideRollers,backRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotyState.setEncoderValue(desiredEncoderValue);
    // backRollers.backRollerOn(speed);
    // sideRollers.sideRollerOn(speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotyState.setEncoderValue(desiredEncoderValue);
    // if(pivoty.getEncoderValue() <desiredEncoderValue-1000 || pivoty.getEncoderValue()>desiredEncoderValue+1000){
    // backRollers.backRollerOn(speed);
    // sideRollers.sideRollerOn(speed);
    // }
    // else{
    //   backRollers.backRollerOff();
    //   sideRollers.sideRollerOff();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!finish){
    pivotyState.setEncoderValue(0);
    // backRollers.backRollerOn(speed);
    // sideRollers.sideRollerOn(speed);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(pivoty.getEncoderValue())>=Math.abs(desiredEncoderValue);
    if( finish){
      return pivoty.getEncoderValue() <= desiredEncoderValue+4000 && pivoty.getEncoderValue() >= desiredEncoderValue-4000;
    }
    else{
    return false;
   }
  }
}
