package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Swerve;
import frc.robot.ArmPositions.Position;
import frc.robot.ArmPositions.Type;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean openLoop;
    
    private PIDController headingController = new PIDController(0.008, 0, 0);

    private Swerve s_Swerve;
    private CommandPS4Controller controller;

    private double kP = 0.01, kI = 0.0, kD = 0;
    private PIDController balancePID = new PIDController(kP, kI, kD);

    /*
    private final int limit = 5;
    private final SlewRateLimiter m_ForwardBackLimit = new SlewRateLimiter(limit);
    private final SlewRateLimiter m_SideSideLimit = new SlewRateLimiter(limit);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(limit);
    */
    private LimelightResults ll;

    /**
     * Driver Control command
     * @param s_Swerve Swerve subsystem
     * @param controller PS4 controller
     * @param openLoop True
     */
    public TeleopSwerve(Swerve s_Swerve, CommandPS4Controller controller, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.openLoop = openLoop;
        
        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);  
        
        s_Swerve.resetEstimator();
    }

    @Override
    public void execute() {
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        double speedMod = (Math.abs(RobotContainer.elevator.getElevateHeight()) > 10)?0.3:1;
        
        if(RobotContainer.stationSelector.getPos() == Position.HYBRID){
            LimelightHelpers.setPipelineIndex(Constants.LimeLightName, 2);
        }else{
            //LimelightHelpers.setPipelineIndex(Constants.LimeLightName, (RobotContainer.stationSelector.getType() == Type.CUBE)?1:0);
            LimelightHelpers.setPipelineIndex(Constants.LimeLightName, 3);
        }
        double x = 0;
        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
        

        if(controller.L1().getAsBoolean()){
            translation = new Translation2d(yAxis*speedMod, xAxis*speedMod).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, speedMod*steerAlign(0, s_Swerve.getYaw().getDegrees()), true, openLoop);
        }else{
            translation = new Translation2d(yAxis*speedMod, xAxis*speedMod).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, rotation*speedMod, true, openLoop);
        }

        if(controller.square().getAsBoolean()){
            s_Swerve.resetEstimator();
        }
        s_Swerve.addVisionTele();
    }


    private double steerAlign(double desiredAngle, double currentAngle){
        currentAngle = s_Swerve.getYaw().getDegrees()%360;
        //Account for flip
        if(desiredAngle - currentAngle > 180){
            desiredAngle = -((360-desiredAngle)+currentAngle);
            currentAngle = 0;
        }

        headingController.setSetpoint(desiredAngle);
        rotation = MathUtil.clamp(headingController.calculate(currentAngle), -0.75, 0.75);
        rotation *= Constants.Swerve.maxAngularVelocity;
        return rotation;
    }

    public final static double[] conePoints = {0.5127,1.6303,2.1891,3.3067,3.8655,4.9831};
    public final static double[] cubePoints = {1.0715,2.7479,4.4243};
    public final static double redPlacePos = 14.85;
    public final static double bluePlacePos = 1.8145;

    public double getClosest(double[] array){
        double closest = array[0];
        for(int i = 1; i < array.length; i++){
            if(Math.abs(array[i]-s_Swerve.getVisionPose().getY()) < 
                Math.abs(closest-s_Swerve.getVisionPose().getY()))
            {
                closest = array[i];
            }
        }
        return closest;
    }

    /*Old Lineup  (redPlacePos-s_Swerve.getVisionPose().getY())*0.01
     * if(controller.L1().getAsBoolean() && RobotContainer.stationSelector.getPos()==Position.HYBRID){
            translation = new Translation2d(yAxis*speedMod, xAxis*speedMod).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, speedMod*steerAlign(180, s_Swerve.getYaw().getDegrees()), true, openLoop);
        }else if(controller.L1().getAsBoolean() && (ll.targetingResults.targets_Retro.length > 0 || ll.targetingResults.targets_Fiducials.length > 0)){
            if(ll.targetingResults.targets_Retro.length > 0){
                x = ll.targetingResults.targets_Retro[0].tx*-0.025;
            }else if(ll.targetingResults.targets_Fiducials.length > 0){
                x = ll.targetingResults.targets_Fiducials[0].tx*-0.025;
            }
            translation = new Translation2d(-yAxis*speedMod, x*speedMod).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, speedMod*steerAlign(180, s_Swerve.getYaw().getDegrees()), false, openLoop);
        }else
     */
}
