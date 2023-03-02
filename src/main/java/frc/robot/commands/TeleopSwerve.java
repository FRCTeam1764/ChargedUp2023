package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {  

    double offsetToleranceProprtion;

    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private RobotState robotState;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, RobotState robotState) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.robotState = robotState;
    }
    
    
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(getTranslation(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble()*.75, SwerveConstants.stickDeadband);
       
       //auto balance if autobalance has been toggled
        if(robotState.swerveState.getStartButton()){
            robotState.swerveState.swerveAutoBalance();
        }
        // System.out.println(translationVal);
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.Swerve.maxSpeed), 
            rotationVal * SwerveConstants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
    double transValue;
    public double getTranslation(){
        if(robotState.swerveState.getSwerveState()){
            transValue = getAutoLevel();
        }
        else{
            transValue = translationSup.getAsDouble();
        }
        return transValue;
    }
    double error;
    double autoLevelPwr;
    public double getAutoLevel(){
       error = -s_Swerve.getNavx().getRoll();
       if(Math.abs(error)<1){
           robotState.swerveState.noSwerveAutoBalance();;
       }
       System.out.println("error" + error);
       autoLevelPwr = -Math.min(error*.02, 1);
        System.out.println(error+ " " +autoLevelPwr);
       return autoLevelPwr;
   }
}