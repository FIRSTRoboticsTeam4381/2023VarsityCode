package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class ArmAngleSubsystem extends SubsystemBase{
    
    private CANSparkMax armTilt1;
    private CANSparkMax armTilt2;
    private RelativeEncoder armTilt1Encoder;
    private RelativeEncoder armTilt2Encoder;
    private AbsoluteEncoder armPivotEncoder;
    private SparkMaxPIDController armTiltPID;

    private TrapezoidProfile.State m_ArmSetPoint = new TrapezoidProfile.State();
    private double anglePos = 0;

    
    public ArmAngleSubsystem(){
        armTilt1 = new CANSparkMax(Constants.IntakeArm.armTilt1CAN, MotorType.kBrushless);
        armTilt2 = new CANSparkMax(Constants.IntakeArm.armTilt2CAN, MotorType.kBrushless);

        armTilt1Encoder = armTilt1.getEncoder();
        armTilt2Encoder = armTilt2.getEncoder();
        armPivotEncoder = armTilt1.getAbsoluteEncoder(Type.kDutyCycle);

        armTilt1Encoder.setPosition((armPivotEncoder.getPosition()-0.5)*245.45);
        armTilt2Encoder.setPosition(0);

        //armTilt2.follow(armTilt1,true);
        armTilt2.setIdleMode(IdleMode.kBrake);
        armTilt2.set(0);

        armTiltPID = armTilt1.getPIDController();
        armTiltPID.setFeedbackDevice(armTilt1Encoder);
        armTiltPID.setP(0.11);
        armTiltPID.setI(0);
        armTiltPID.setD(0.0015);
        armTiltPID.setFF(0.0002);
        armTiltPID.setOutputRange(-1, 1);
        armTilt1.setIdleMode(IdleMode.kBrake);

        armTiltPID.setSmartMotionMaxAccel(2 * Constants.IntakeArm.ArmTiltRatio, 0);
    }

    public void setArmAngle(double angle){
        anglePos = angle;
        //armTiltPID.setReference(angle, ControlType.kPosition);
    }

    public double getArmAngle(){
        return armTilt1Encoder.getPosition(); //*armConversionfactor gear ratios and stuff
    }

    public double getArmVelocity(){
        return armTilt1Encoder.getVelocity(); //*arm velocity conversion
    }

    public void resetArm(){
        armTilt1Encoder.setPosition((armPivotEncoder.getPosition()-0.5)*245.45);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Tilt Encoder", Conversions.armEncoderToDegrees(armTilt1Encoder.getPosition()));
        SmartDashboard.putNumber("Arm Absolute", armPivotEncoder.getPosition());
        SmartDashboard.putNumber("Arm Angle Setpoint", Conversions.armEncoderToDegrees(anglePos));

        TrapezoidProfile armProfile = new TrapezoidProfile(
            new Constraints(3000, 500),//Could use a little less accel
            new State(anglePos, 0),
            m_ArmSetPoint
        );
        m_ArmSetPoint = armProfile.calculate(0.02);
        armTiltPID.setReference(m_ArmSetPoint.position, ControlType.kPosition);
    }
}
