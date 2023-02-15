// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */

  // SparkMaxAlternateEncoder intakeMotor;
  double intakePower;
  CANSparkMax intakeMotor;
  int timer;
  RelativeEncoder encoder;
  CANSparkMax sideRollers;
  CANSparkMax backRollers;
  DigitalInput color;
  int timerTwo;
  
  public intakeSubsystem() {
  // this.intakeMotor = new SparkMaxAlternateEncoder(sparkMax, 42);
    intakeMotor = new CANSparkMax(Constants.INTAKE_OPENER_MOTOR, MotorType.kBrushless);
     sideRollers = new CANSparkMax(1,MotorType.kBrushless);
     backRollers = new CANSparkMax(1, MotorType.kBrushless);
    
     color = new DigitalInput(5); //5 is just a place holder

    // We got color!!! :D
    
  }
  public void intakeClose(){
    sideRollers.set(0.5);
    if (color.get()) {
      backRollers.set(0.5);
    } else {
      backRollers.set(0);
    }

    intakeMotor.set(.2);
    if (timer > 30) {
      encoder.setPosition(0);
      backRollers.set(0);
      sideRollers.set(0);
    }
    if (timer < 30);
    timer += 1;   
  }
  
  public void intakeOpen(){
    timer = 0;
    backRollers.set(0);
    sideRollers.set(0);
    while(encoder.getPosition()<69){
      intakeMotor.set(-0.2);
    }
  }
  //may need to swap this to work off of while true, also create command
  public void intakeShoot(){
    intakeMotor.set(0.1);
    sideRollers.set(-1);
    backRollers.set(0);
  if (timerTwo > 20) {
    intakeMotor.set(0);
    sideRollers.set(0);
    intakeOpen();
    }
    if (timerTwo < 20){
      timerTwo += 1;
    }
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
  }
}
