// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

/** Add your docs here. */
public class PivotyState {
    int encoderValue;
    PivotyState(){
    }
    
    public void setEncoderValue(int desiredValue){
        encoderValue = desiredValue;
    }

    public int getEncoderValue(){
        return encoderValue;
    }

}
