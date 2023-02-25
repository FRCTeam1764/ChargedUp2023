// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.state.IntakeState;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */

  // SparkMaxAlternateEncoder intakeMotor;
  double intakePower;
  public int timer;

  CANSparkMax sideRollers;
  CANSparkMax backRollers;
  DigitalInput color1;
  DigitalInput color2;
  int timerTwo;
  IntakeState intakeState;

  
  public intakeSubsystem(IntakeState intakeState) {
  // this.intakeMotor = new SparkMaxAlternateEncoder(sparkMax, 42);

     sideRollers = new CANSparkMax(Constants.SIDE_INTAKE_MOTOR,MotorType.kBrushless);
     backRollers = new CANSparkMax(Constants.BACK_INTAKE_MOTOR, MotorType.kBrushless);
    
     color1 = new DigitalInput(Constants.COLOR_SENSOR_1);
     color2 = new DigitalInput(Constants.COLOR_SENSOR_2);
     this.intakeState = intakeState;
    // We got color!!! :D
    
  }
  //intake - has built in color sensor, intakes ball/cone depending on it
  public void intakeClose(double intakeSpeed){
    sideRollers.set(intakeSpeed);
    if (color1.get() && color2.get()) {
      backRollers.set( 0.5); //use backrollers when cone
    } else {
      backRollers.set(0);
    }

    if ((color1.get() && color2.get()) || (!color1.get() && color2.get()))
    intakeState.setIntakeClose(true);
    if (timer > 30) {

      backRollers.set(0);
      sideRollers.set(0);
      
    }
    if (timer < 30);
    timer += 1;   
  }
  
  public void intakeOpen(double intakeSpeed){
      timer = 0;
      if(timerTwo<20){
        sideRollers.set(intakeSpeed);
        timerTwo+=1;
      }
      else{
      intakeState.setIntakeClose(false);
      backRollers.set(0);
      sideRollers.set(0);
      }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
  }
  public int getTimer(){
    return timer;
  }
  public int getTimer2(){
    return timerTwo;
  }

  public boolean getColor1(){
    return color1.get();
  }

  public boolean getColor2(){
    return color2.get();
  }
   
}