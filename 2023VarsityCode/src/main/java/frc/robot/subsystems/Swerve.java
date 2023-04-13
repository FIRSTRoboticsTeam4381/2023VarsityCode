package frc.robot.subsystems;
import java.lang.reflect.Field;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private LimelightResults ll;
    private Field2d m_field;
    private SwerveDrivePoseEstimator estimator;

    private boolean fieldRel = true;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.DriveCANBus);
        gyro.setYaw(0);
        gyro.configMountPoseRoll(-1.35);
        zeroGyro(180);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
        estimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getYaw(),
            getPositions(), 
            new Pose2d(0,0, Rotation2d.fromDegrees(0)),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0005, 0.0005, 0.0005), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, 0.05)
        );
        
        // if(ll.targetingResults.targets_Fiducials.length>1){
        //     estimator.addVisionMeasurement(ll.targetingResults.getBotPose2d_wpiBlue(), ll.targetingResults.latency_capture);
        // }
        //resetToVision();

        m_field = new Field2d();
        m_field.setRobotPose(estimator.getEstimatedPosition());
        SmartDashboard.putData("Field", m_field);

    }

    /**
     * Function used to actually drive the robot
     * @param translation XY drive values
     * @param rotation Rotation value
     * @param fieldRelative True -> fieldOriented
     * @param isOpenLoop True
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        /*
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(DriftCorrection.driftCorrection(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation), 
                                swerveOdometry.getPoseMeters())
                                );
        */
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /**
     * @return XY of robot on field
     */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }


    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d yaw) {
        estimator.resetPosition(yaw, getPositions(), pose);
    }
    
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Use to reset angle to certain known angle or to zero
     * @param angle Desired new angle
     */
    public void zeroGyro(double angle){
        gyro.setYaw(angle);
        gyro.configMountPoseRoll(-1.35);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * 
     * @return The roll of the robot with forward being negative
     */
    public double getRoll(){
        return gyro.getRoll();
    }

    /**
     * TODO {negative or positive}. Used for auto-balance
     * @return The roll of the robot with forward being negative
     */
    public double getPitch(){
        return gyro.getPitch();
    }
        
    public void setFieldRel(boolean rel){
        fieldRel = rel;
    }
    public boolean getFieldRel(){
        return fieldRel;
    }

    /*
    private Pose2d newPose;
    public void addVision(){
        if(ll.targetingResults.targets_Fiducials.length > 0){
            if(ll.targetingResults.targets_Fiducials[0].ta > 0.01){
                if(DriverStation.getAlliance() == Alliance.Red){
                    newPose = new Pose2d(16.54-ll.targetingResults.getBotPose2d_wpiBlue().getX(), ll.targetingResults.getBotPose2d_wpiBlue().getY(), ll.targetingResults.getBotPose2d_wpiBlue().getRotation().rotateBy(Rotation2d.fromDegrees(180)));
                }else{
                    newPose = ll.targetingResults.getBotPose2d_wpiBlue();
                }
                estimator.addVisionMeasurement(newPose, Timer.getFPGATimestamp());
            }
        }
    }
    */
    private Pose2d tempPose;
    public void addVision(){
        if(ll.targetingResults.targets_Fiducials.length > 0){
            if(ll.targetingResults.targets_Fiducials[0].ta > 0.018){
                if(DriverStation.getAlliance() == Alliance.Blue){
                    estimator.addVisionMeasurement(ll.targetingResults.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp());
                 }else{
                     tempPose = new Pose2d(
                         16.54-ll.targetingResults.getBotPose2d_wpiBlue().getX(),
                         8.02-ll.targetingResults.getBotPose2d_wpiBlue().getY(),
                         ll.targetingResults.getBotPose2d_wpiBlue().getRotation().rotateBy(Rotation2d.fromDegrees(180))//times(-1).
                         );
                     estimator.addVisionMeasurement(tempPose, Timer.getFPGATimestamp());                
                 }
            }
        }
    }

    public void resetToVision(){
        if(ll.targetingResults.targets_Fiducials.length > 0){
            if(ll.targetingResults.targets_Fiducials[0].ta > 0.01){
                if(DriverStation.getAlliance() == Alliance.Blue){
                    estimator.addVisionMeasurement(ll.targetingResults.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp());
                 }else{
                     tempPose = new Pose2d(
                         16.54-ll.targetingResults.getBotPose2d_wpiBlue().getX(),
                         8.02-ll.targetingResults.getBotPose2d_wpiBlue().getY(),
                         ll.targetingResults.getBotPose2d_wpiBlue().getRotation().rotateBy(Rotation2d.fromDegrees(180))//times(-1).
                         );
                     estimator.addVisionMeasurement(tempPose, Timer.getFPGATimestamp());                
                 }
            }
        }
    }

    @Override
    public void periodic(){
        estimator.update(getYaw(), getPositions());
        addVision();
        SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
        SmartDashboard.putNumber("Gyro Roll", getRoll());

        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Temp", mod.getTemp(1));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Temp", mod.getTemp(2));
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Setpoint", mod.getDesired());
        }
        

        SmartDashboard.putString("XY Coord", "(" + getPose().getX() + ", " + getPose().getY() + ")");

        m_field.setRobotPose(getPose());

        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
    }
}