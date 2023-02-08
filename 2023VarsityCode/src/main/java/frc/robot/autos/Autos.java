package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Balance;

public final class Autos {

    /**
     * Events to be used in all Autos built with pathplanner
     */
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("example1", Commands.print("Example 1 triggered")),
        Map.entry("example2", Commands.print("Example 2 triggered")),
        Map.entry("example3", Commands.print("Example 3 triggered")),
        Map.entry("lime", new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3))),
        Map.entry("stop", new InstantCommand(() -> RobotContainer.s_Swerve.drive(new Translation2d(0,0), 0, true, true))),
        Map.entry("balance", new Balance(RobotContainer.s_Swerve))
    ));

    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        RobotContainer.s_Swerve::getPose, // Pose2d supplier
        RobotContainer.s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        RobotContainer.s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        RobotContainer.s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
    public static PPSwerveControllerCommand followTrajectory(PathPlannerTrajectory traj){
        return new PPSwerveControllerCommand(
            traj, 
            RobotContainer.s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(5.0, 0, 0),
            new PIDController(5.0, 0, 0),
            new PIDController(0.5, 0, 0),
            RobotContainer.s_Swerve::setModuleStates,
            true,
            RobotContainer.s_Swerve
        );
    }
    

    public static PathPlannerTrajectory goToPoint(double pointX, double pointY, Pose2d swervePose){
        return PathPlanner.generatePath(
            new PathConstraints(2, 1.5),
            new PathPoint(swervePose.getTranslation(), swervePose.getRotation(), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(pointX, pointY), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
            );
    }

    /*
     * Possible waypoint angle calculation
     *      90+Math.tan(
            (swervePose.getX()-RobotContainer.stationSelector.getStagePoint()[0])/
            (swervePose.getY()-RobotContainer.stationSelector.getStagePoint()[1])))
     */

    public static PathPlannerTrajectory runToPlace(Pose2d swervePose){
        double adj = (swervePose.getY() > RobotContainer.stationSelector.getStagePoint()[1])? -1.0:1.0;
        double yaw = RobotContainer.s_Swerve.getYaw().getDegrees();
        return PathPlanner.generatePath(
            new PathConstraints(2, 1.5),
            new PathPoint(swervePose.getTranslation(), 
                Rotation2d.fromDegrees(adj*90), 
                0),
            new PathPoint(new Translation2d(
                RobotContainer.stationSelector.getStagePoint()[0], 
                RobotContainer.stationSelector.getStagePoint()[1]), 
                Rotation2d.fromDegrees(adj*45), 
                Rotation2d.fromDegrees(180-yaw)),
            new PathPoint(new Translation2d(
                RobotContainer.stationSelector.getPlacePoint()[0], 
                RobotContainer.stationSelector.getPlacePoint()[1]), 
                Rotation2d.fromDegrees(-180), 
                Rotation2d.fromDegrees(180-yaw))
            );
    }
    
    /**
     * Auto to test PathPlanner
     * @return
     */
    public static Command exampleAuto(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("PathPlannerTest", 
            new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
    }

    public static Command singleCone(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("FullAuto",
            new PathConstraints(3, 1))).andThen(
                balanceCommad()
            );
    }

    public static Command blindMike(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("TheBlindingOfMicheal", 
            new PathConstraints(2, 1)));
    }

    public static Command goToPoint(double pointX, double pointY){;
        return followTrajectory(goToPoint(pointX, pointY, RobotContainer.s_Swerve.getPose()));
    }

    public static Command balanceCommad(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("BalanceTest",
            new PathConstraints(2, 1)));
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static Command none(){
        return Commands.none();
    }

}
