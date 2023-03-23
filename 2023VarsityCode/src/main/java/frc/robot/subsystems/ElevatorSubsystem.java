package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase{

    private CANSparkMax armExtend;   
    private RelativeEncoder armExtensionEncoder;
    private SparkMaxPIDController armExtendPID;
   
    private double elevatePos = 0;
    private TrapezoidProfile.State m_ElevatorSetPoint = new TrapezoidProfile.State();


    public ElevatorSubsystem(){
        armExtend = new CANSparkMax(Constants.IntakeArm.armExtensionCAN, MotorType.kBrushless);
        armExtensionEncoder = armExtend.getEncoder();

        armExtendPID = armExtend.getPIDController();
        armExtendPID.setP(0.4);
        armExtendPID.setI(0);
        armExtendPID.setD(0.0004);
        armExtendPID.setFF(0.00017);
        armExtendPID.setOutputRange(-0.75, 0.75);
        armExtendPID.setSmartMotionMaxAccel(2*9.4,0);
        armExtend.setIdleMode(IdleMode.kBrake);
    }


    public double getElevateHeight(){
        return armExtensionEncoder.getPosition();
    }

    public double getElevateVelocity(){
        return armExtensionEncoder.getVelocity();
    }    

    public void setElevator(double height){
        elevatePos = height;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Extend Encoder", armExtensionEncoder.getPosition());     
        SmartDashboard.putNumber("Arm Extend Setpoint", elevatePos);
  
        TrapezoidProfile elevateProfile = new TrapezoidProfile(
            new Constraints(2000, 300),//Little more accel, higher power/velocity
            new State(elevatePos, 0),
            m_ElevatorSetPoint
        );
        m_ElevatorSetPoint = elevateProfile.calculate(0.02);
        armExtendPID.setReference(m_ElevatorSetPoint.position, ControlType.kPosition);
    }
}
