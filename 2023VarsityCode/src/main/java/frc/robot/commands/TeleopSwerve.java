package frc.robot.commands;

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
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeArm.Type;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean openLoop;
    
    /*
    private boolean absoluteRotation = false;
    private double desiredAngle;
    private PIDController headingController = new PIDController(0.008, 0, 0);
    */

    private Swerve s_Swerve;
    private CommandPS4Controller controller;

    private double kP = 0.015, kI = 0.0, kD = 0.007;
    private PIDController balancePID = new PIDController(kP, kI, kD);

    private final Field2d m_field = new Field2d();
    private Pose2d startPose = new Pose2d(Units.inchesToMeters(177), Units.inchesToMeters(214), Rotation2d.fromDegrees(0));
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

        SmartDashboard.putData("Field", m_field);
        
        m_field.setRobotPose(startPose);    
        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);    
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

        //Field Steer
        /*
        controller.share().onTrue(new InstantCommand(() -> absoluteRotation = !absoluteRotation));
        SmartDashboard.putBoolean("Absolute Rotation", absoluteRotation);
        if(absoluteRotation){
            if(Math.abs(rAxisX) + Math.abs(rAxisY) >=0.8){
                desiredAngle = new Translation2d(-rAxisY, -rAxisX).getAngle().getDegrees() + 180;
            }
            
            double currentAngle = s_Swerve.getYaw().getDegrees()%360;
            //Account for flip
            if(desiredAngle - currentAngle > 180){
                desiredAngle = -((360-desiredAngle)+currentAngle);
                currentAngle = 0;
            }

            headingController.setSetpoint(desiredAngle);
            rotation = MathUtil.clamp(headingController.calculate(currentAngle), -0.75, 0.75);
            rotation *= Constants.Swerve.maxAngularVelocity;
        }else{
            
            desiredAngle = 0;
        }
        SmartDashboard.putNumber("Desired Drive Heading", desiredAngle);
        */

        //Balancer
        boolean fieldRelative = true;
        if(controller.cross().getAsBoolean()){
            xAxis = 0.0;
            rotation = 0.0;
            yAxis = -MathUtil.clamp(balancePID.calculate(s_Swerve.getRoll(), 0.0), -0.12, 0.12);
            fieldRelative = false;
        }

        LimelightHelpers.setPipelineIndex(Constants.LimeLightName, (RobotContainer.stationSelector.getType() == Type.CUBE)?2:1);
        double x = 0;
        if(controller.L1().getAsBoolean()){
            ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
            if(ll.targetingResults.targets_Retro.length > 0){
                x = ll.targetingResults.targets_Retro[0].tx*-0.025;
                translation = new Translation2d(yAxis, x).times(Constants.Swerve.maxSpeed);
                s_Swerve.drive(translation, rotation, false, openLoop);
            }else if(ll.targetingResults.targets_Fiducials.length > 0){
                x = ll.targetingResults.targets_Fiducials[0].tx*-0.025;
                translation = new Translation2d(yAxis, x).times(Constants.Swerve.maxSpeed);
                s_Swerve.drive(translation, rotation, false, openLoop);
            }
        }else if(!RobotContainer.arm.LOCKOUT){
            translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
            s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        }else{
            s_Swerve.drive(new Translation2d(0,0), rotation*0.5, true, true);
        }

        SmartDashboard.putNumber("X", x);

        m_field.setRobotPose(s_Swerve.getPose());
    }
}
