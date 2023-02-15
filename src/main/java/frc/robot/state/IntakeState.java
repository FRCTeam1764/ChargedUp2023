// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

/** Add your docs here. */
public class IntakeState {
    boolean intakeClose;
    public IntakeState(){
        intakeClose = true;
    }
    public void setIntakeClose(boolean intakeClose){
        this.intakeClose = intakeClose;
    }
    public boolean getIntakeClose(){
        return intakeClose;
    }
}
