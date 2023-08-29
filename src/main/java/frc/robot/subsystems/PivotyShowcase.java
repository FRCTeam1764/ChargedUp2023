// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.LazyTalonFX;

public class PivotyShowcase extends SubsystemBase {
  /** Creates a new PivotyShowcase. */
  public double encoderOffset;
  private LazyTalonFX pivotyMotor1;
  private LazyTalonFX pivotyMotor2;
    public DigitalInput breakBeamOne;
    public boolean upOrDown;


  public PivotyShowcase() {
    pivotyMotor1 = new LazyTalonFX(Constants.PIVOTY_MOTOR.id, Constants.PIVOTY_MOTOR.busName);
    pivotyMotor2 = new LazyTalonFX(Constants.PIVOTY_MOTOR_2.id, Constants.PIVOTY_MOTOR_2.busName);
     breakBeamOne = new DigitalInput(Constants.PIVOTY_BREAK_BEAM);

    encoderOffset = 115000;
  }

  public void PivotyUp(){
pivotyMotor1.setVoltage(.2);
pivotyMotor2.setVoltage(.2);
  }
  public void PivotyDown(){
    pivotyMotor1.setVoltage(-.2);
    pivotyMotor2.setVoltage(-.2);
      }
      public void PivotyOff(){
        pivotyMotor1.setVoltage(0);
        pivotyMotor2.setVoltage(0);
          }



     public void zeroEncoder(){
     pivotyMotor1.getSensorCollection().setIntegratedSensorPosition(0.0,0);
     pivotyMotor2.getSensorCollection().setIntegratedSensorPosition(0.0,0);
   }
   public double getEncoderRadiansIntake(){
     return ((pivotyMotor2.getSelectedSensorPosition()-encoderOffset)/(224*2048))*(2*Math.PI);
   }
      public boolean getBrkBeam(){
    return breakBeamOne.get();
   }
  @Override


  public void periodic() {
if (pivotyMotor2.getSelectedSensorPosition()-encoderOffset <=0){
upOrDown = true;
  } 
  else if(pivotyMotor2.getSelectedSensorPosition()-encoderOffset >=120000){
upOrDown = false;
  }

  if (upOrDown == true ){
PivotyUp();
  }else{
PivotyDown();
  }

      // This method will be called once per scheduler run

}
}
