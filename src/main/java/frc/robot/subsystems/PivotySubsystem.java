package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class PivotySubsystem extends SubsystemBase {
  private TalonFX motor1;
  private TalonFX motor2;
  private DigitalInput pivotyBreakBeam; 
  

  public PivotySubsystem(DigitalInput pivotyBreakBeam) {
    this.motor1 = new TalonFX(Constants.PIVOTY_MOTOR1);
    this.motor2 = new TalonFX(Constants.PIVOTY_MOTOR2);
    this.pivotyBreakBeam = pivotyBreakBeam;

  }

  public void PivotyOn(double pivotySpeed){
    if(pivotyBreakBeam.get()){
      motor1.set(ControlMode.PercentOutput, pivotySpeed);
      motor2.set(ControlMode.PercentOutput, pivotySpeed);
  } else{
      motor1.set(ControlMode.PercentOutput, 0.0);
      motor2.set(ControlMode.PercentOutput, 0.0);
  }
}

  public void PivotyOff(){
    motor1.set(ControlMode.PercentOutput, 0.0);
    motor2.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
;