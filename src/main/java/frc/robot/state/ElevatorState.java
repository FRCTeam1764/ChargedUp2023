// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

/** Add your docs here. */
public class ElevatorState {
    double encoderValue;
    double velocity;
    ElevatorState(){
    }
    
    public void setEncoderValue(double desiredValue){
        encoderValue = desiredValue;
    }


    public void setVelocity(double Desiredvelocity){
        velocity = Desiredvelocity;
    }


    public double getEncoderValue(){
        return encoderValue;
    }

    public double getVelocity(){
        return velocity;
    }
}
