// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.state;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class RobotState {
    public LimelightState limelightState;

    public SwerveState swerveState;
    public PivotyState pivotyState;
    public ElevatorState elevatorState;
    public RobotState(Joystick driver) {
        this.limelightState = new LimelightState();
        this.swerveState = new SwerveState(driver);
        this.pivotyState = new PivotyState();
        this.elevatorState = new ElevatorState();
    }
}