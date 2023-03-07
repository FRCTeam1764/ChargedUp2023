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
        this.limelightState = robotState.limelightState;
    }

    /*
     * checks if limelight is on + target is in sight
     * moves to target
     */
    public double moveLeftOrRight() {
        if (limelightState.getLimelightState() == true && limelightSubsystem.isThereTarget ==1) {
            strafeVal = MathUtil.applyDeadband(limelightSubsystem.whereToMove(), SwerveConstants.stickDeadband);
        } else {
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.stickDeadband);
        }
        return strafeVal;
    }
/*
 * returns transitional and rotational values based on gyro rotation to balance
 * preferibly robot should be straight
 */
    public double[] autoBalance(DoubleSupplier yaxis,DoubleSupplier xaxis,AHRS ahrs){

        boolean autoBalanceXMode = true;
        boolean autoBalanceYMode = true;

        Double xAxisRate            = xaxis.getAsDouble();
        Double yAxisRate            = yaxis.getAsDouble();
        double pitchAngleDegrees    = ahrs.getPitch();
        double rollAngleDegrees     = ahrs.getRoll();
        
        if ( !autoBalanceXMode && 
             (Math.abs(pitchAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if ( !autoBalanceYMode && 
             (Math.abs(pitchAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
        }
        else if ( autoBalanceYMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
        }
        
        // Control drive system automatically, 
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle
        
        if ( autoBalanceXMode ) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
        if ( autoBalanceYMode ) {
            double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
            yAxisRate = Math.sin(rollAngleRadians) * -1;
        }

        double[] aidenSucks = {xAxisRate,yAxisRate};
        return aidenSucks; // curse you sawyer
      //  myRobot.mecanumDrive_Cartesian(xAxisRate, yAxisRate, stick.getTwist(),0);
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