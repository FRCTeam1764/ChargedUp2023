// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.libraries.internal.RevThroughBoreEncoder;
// import frc.robot.constants.GlobalConstants.IntakeConstants;
// import frc.robot.trobot5013lib.RevThroughBoreEncoder;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class Intake extends SubsystemBase {
    private final CANSparkMax m_flexMotor = new CANSparkMax(Constants.WRIST_MOTOR.id, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR.id, MotorType.kBrushless);
    private final PIDController m_flexPIDController = new PIDController(1.1, 0, 0.05);
    private final RevThroughBoreEncoder m_angleEncoder = new RevThroughBoreEncoder(Constants.WRIST_ANGLE_ENCODER);

    private ArmFeedforward m_feedForward = new ArmFeedforward(0.8, 1.1,1); 
    private PivotySubsystem pivoty;


    public Intake(PivotySubsystem pivoty) {
        super();
        m_intakeMotor.restoreFactoryDefaults();
        m_flexMotor.restoreFactoryDefaults();
        m_flexMotor.setInverted(true);
        m_flexMotor.setIdleMode(IdleMode.kBrake);
        m_flexPIDController.enableContinuousInput(0, 2 * Math.PI);
        m_angleEncoder.setOffset(Rotation2d.fromDegrees(-116));
        m_angleEncoder.setInverted(true);
        this.pivoty = pivoty;


    } 

    public double getAngleRadians() {
        return ((m_angleEncoder.getAngle()).getRadians());
    }

    public void pickUpCone() {
        m_intakeMotor.set(-0.5);
    }

    public void pickUpCube() {
        m_intakeMotor.set(0.5);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }
    
    public void flexClosedLoop(double velocity) {
        SmartDashboard.putNumber("Flex velocity", velocity);
        double feedForward = m_feedForward.calculate(getGroundRelativeWristPossitionRadians() ,velocity); //calculate feed forward
        m_flexMotor.setVoltage(feedForward);
    }

    public double getGroundRelativeWristPossitionRadians(){
        return pivoty.getEncoderRadians() + getAngleRadians();
    }
//m_angleEncoder.isConnected() && pivoty.isArmEncoderConnected() && 

    @Override
    public void periodic(){
        SmartDashboard.putNumber("WristAngle",(m_angleEncoder.getAngle()).getDegrees());
        SmartDashboard.putNumber("WriseAngleGround",Units.radiansToDegrees(getGroundRelativeWristPossitionRadians())); 
        if (
           pivoty.getEncoderRadiansButForReal() < Units.degreesToRadians(85) || pivoty.getEncoderRadiansButForReal() > Units.degreesToRadians(337) ){
            m_flexPIDController.setTolerance(Rotation2d.fromDegrees(2).getRadians());
            m_flexPIDController.setSetpoint(Rotation2d.fromDegrees(17).getRadians());
            flexClosedLoop(m_flexPIDController.calculate(getGroundRelativeWristPossitionRadians()));
        }
    }
    
 }
//there was a "isconnected" for m_angleencoder