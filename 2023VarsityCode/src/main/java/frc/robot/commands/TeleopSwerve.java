package frc.robot.commands;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.ArmPositions.Type;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean openLoop;
    
    private PIDController headingController = new PIDController(0.008, 0, 0);

    private Swerve s_Swerve;
    private CommandPS4Controller controller;

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
        
        //ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);    
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

        /*
        LimelightHelpers.setPipelineIndex(Constants.LimeLightName, (RobotContainer.stationSelector.getType() == Type.CUBE)?2:1);
        double x = 0;
        if(controller.L1().getAsBoolean()){
            ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
            if(ll.targetingResults.targets_Retro.length > 0){
                x = ll.targetingResults.targets_Retro[0].tx*-0.025;
                translation = new Translation2d(-yAxis, x).times(Constants.Swerve.maxSpeed);
                s_Swerve.drive(translation, steerAlign(180, s_Swerve.getYaw().getDegrees()), false, openLoop);
            }else if(ll.targetingResults.targets_Fiducials.length > 0){
                x = ll.targetingResults.targets_Fiducials[0].tx*-0.025;
                translation = new Translation2d(-yAxis, x).times(Constants.Swerve.maxSpeed);
                s_Swerve.drive(translation, steerAlign(180, s_Swerve.getYaw().getDegrees()), false, openLoop);
            }
        }else{*/
        if(controller.L1().getAsBoolean()){
            translation = new Translation2d(-yAxis, -xAxis).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, -steerAlign(180, s_Swerve.getYaw().getDegrees()), true, openLoop);
        }else{
            translation = new Translation2d(-yAxis, -xAxis).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, -rotation, true, openLoop);
        }
        
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
}