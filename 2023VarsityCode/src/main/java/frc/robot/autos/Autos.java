package frc.robot.autos;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Balance;
import frc.robot.subsystems.IntakeArm.Position;
import frc.robot.subsystems.IntakeArm.Type;

public final class Autos {

    /**
     * Events to be used in all Autos built with pathplanner
     */
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("lime", new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3))),
        Map.entry("stop", new InstantCommand(() -> RobotContainer.s_Swerve.drive(new Translation2d(0,0), 0, true, true))),
        Map.entry("balance", new Balance(RobotContainer.s_Swerve)),
        Map.entry("BackwardsCube", new InstantCommand(() -> RobotContainer.arm.setState(Position.AUTOCUBE))),
        Map.entry("Transit", new InstantCommand(() -> RobotContainer.arm.setState(Position.TRANSIT))),
        Map.entry("Cone", new InstantCommand(() -> RobotContainer.arm.setState(Position.UPCONE))),
        Map.entry("SetCube", new InstantCommand(() -> RobotContainer.stationSelector.setType(Type.CUBE))),
        Map.entry("HighPlace", Commands.run(() -> RobotContainer.arm.setState(Position.HIGHPLACE))
                                    .until(() -> RobotContainer.arm.getIntakeEncoder() > RobotContainer.arm.intakePlacePos()+3)
                                    .andThen(Commands.run(() -> RobotContainer.arm.setState(Position.TRANSIT))
                                    .until(() -> Math.abs(RobotContainer.arm.getArmAngle()) < 5))),
        Map.entry("CubePlace", Commands.run(() -> RobotContainer.arm.setState(Position.SHOOTHIGHCUBE))
                                    .until(() -> RobotContainer.arm.getIntakeEncoder() > RobotContainer.arm.intakePlacePos()+3)
                                    .andThen(Commands.run(() -> RobotContainer.arm.setState(Position.TRANSIT))
                                    .until(() -> Math.abs(RobotContainer.arm.getArmAngle()) < 5))),
        Map.entry("MidPlace", Commands.run(() -> RobotContainer.arm.setState(Position.SHOOTMIDCUBE))
                                    .until(() -> RobotContainer.arm.getIntakeEncoder() > RobotContainer.arm.intakePlacePos()+3)
                                    .andThen(Commands.run(() -> RobotContainer.arm.setState(Position.TRANSIT))
                                    .until(() -> Math.abs(RobotContainer.arm.getArmAngle()) < 5)))
    ));

    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        RobotContainer.s_Swerve::getPose, // Pose2d supplier
        RobotContainer.s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(6.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.8, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        RobotContainer.s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        RobotContainer.s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
    

    public static Command threePiece(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ThreePiece",
            new PathConstraints(4, 3)));
    }

    public static Command coneBalance(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("ConePark",
            new PathConstraints(4, 3)));
    }

    
    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static Command none(){
        return Commands.none();
    }

}
