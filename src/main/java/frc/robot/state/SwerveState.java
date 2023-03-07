// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class SwerveState {
    boolean swerveAutoBalance;
    private final JoystickButton toggleDriveTrainAutoBalance;
    public SwerveState(Joystick driver){
        swerveAutoBalance = false;
        toggleDriveTrainAutoBalance =  new JoystickButton(driver, XboxController.Button.kStart.value);
    }
    public void swerveAutoBalance(){
        swerveAutoBalance = true;
    }
    public void noSwerveAutoBalance(){
        swerveAutoBalance = false;
    }
    public boolean getSwerveState(){
        return swerveAutoBalance;
    }
    public boolean getStartButton(){
        return toggleDriveTrainAutoBalance.getAsBoolean();
    }
}