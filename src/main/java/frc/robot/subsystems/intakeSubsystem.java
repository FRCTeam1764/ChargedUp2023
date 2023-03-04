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
  DigitalInput color;
  int timerTwo;
  IntakeState intakeState;

  
  public intakeSubsystem(IntakeState intakeState) {
  // this.intakeMotor = new SparkMaxAlternateEncoder(sparkMax, 42);

     sideRollers = new CANSparkMax(Constants.SIDE_INTAKE_MOTOR.id,MotorType.kBrushless);
     backRollers = new CANSparkMax(Constants.BACK_INTAKE_MOTOR.id, MotorType.kBrushless);
    
     color = new DigitalInput(Constants.COLOR_SENSOR);//there are two color sensors, only reading 1 rn
     this.intakeState = intakeState;
    // We got color!!! :D
    
  }
  //intake - has built in color sensor, intakes ball/cone depending on it
  public void intakeClose(double intakeSpeed){

    // sideRollers.set(intakeSpeed);
    // if (color.get()) {
    //   backRollers.set(0.5); //use backrollers when cone
    // } else {
    //   backRollers.set(0);
    // }

    // intakeState.setIntakeClose(true);
    // if (timer > 30) {

    //   backRollers.set(0);
    //   sideRollers.set(0);
      
    // }
    // if (timer < 30);
    // timer += 1;   

    backRollers.set(intakeSpeed);
    sideRollers.set(intakeSpeed);

    System.out.println(color.get());

    }
// intakeState.setIntakeClose(true);
//    if (timer > 30) {
//        backRollers.set(0);
//        sideRollers.set(0);
//    }

// timer += 1;  


//   }
  
  public void intakeOpen(){

      // intakeState.setIntakeClose(false);
      backRollers.set(0);
      sideRollers.set(0);

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

  public boolean getColor(){
    return color.get();
  }
   
}
