package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
//import frc.robot.LimelightResults;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private LimelightResults ll;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.DriveCANBus);
        gyro.setYaw(0);
        gyro.configMountPoseRoll(-1.35);
        zeroGyro(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
        

        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
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
        return swerveOdometry.getPoseMeters();
    }


    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d yaw) {
        swerveOdometry.resetPosition(yaw, getPositions(), pose);
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
    
    /*
    public double align(){
        return
    }
*/

    public Pose2d limePose(){
        Pose2d newPose = new Pose2d(
            ll.targetingResults.getBotPose2d().getX()+8.27,
            ll.targetingResults.getBotPose2d().getY()+4.01,
            ll.targetingResults.getBotPose2d().getRotation()
        );
        return newPose;
    }

    public void autoReset(){
        ll = LimelightHelpers.getLatestResults(Constants.LimeLightName);
        if(ll.targetingResults.targets_Fiducials.length > 1 || (ll.targetingResults.targets_Fiducials.length > 0 && ll.targetingResults.targets_Fiducials[0].ta > 1)){
            resetOdometry(limePose());
            zeroGyro((DriverStation.getAlliance() == Alliance.Red)
                ?
                ll.targetingResults.getBotPose2d_wpiRed().getRotation().getDegrees()
                :
                ll.targetingResults.getBotPose2d_wpiBlue().getRotation().getDegrees()
                );
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getPositions());
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
    }
}