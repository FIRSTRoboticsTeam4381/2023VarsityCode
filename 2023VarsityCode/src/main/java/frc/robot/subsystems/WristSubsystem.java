package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.ArmPositions;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.ArmPositions.Position;

public class WristSubsystem extends SubsystemBase{

    private double wristPos = Conversions.degreesToWristEncoder(ArmPositions.getArmState(Position.TRANSIT)[2]);
    private double intakeSet = 0;
    private double wristFF = 0.115625;
    private double coneWristFF = 0.1265625;
    private PIDController wristPID;
    private WPI_TalonSRX wristTilt;
    private AbsoluteEncoder wristAbsolute;
    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private TrapezoidProfile.State m_WristSetPoint = new TrapezoidProfile.State();

    private Joystick testJoystick = new Joystick(3);

    
    public WristSubsystem(){
        intake = new CANSparkMax(Constants.IntakeArm.intakeCAN, MotorType.kBrushless);
        intake.setIdleMode(IdleMode.kBrake);
        intake.setSmartCurrentLimit(40);

        wristTilt = new WPI_TalonSRX(Constants.IntakeArm.wristAngleCAN);

        wristTilt.configFactoryDefault();
        wristTilt.setNeutralMode(NeutralMode.Brake);
        wristTilt.configPeakOutputForward(1);
        wristTilt.configPeakOutputReverse(-1);
        
        wristAbsolute = intake.getAbsoluteEncoder(Type.kDutyCycle);
        wristPID = new PIDController(4, 0.00, 0.1);
        
        intake.set(0);
        intakeEncoder = intake.getEncoder();
        intakeEncoder.setPosition(0);
    }


    public void setWristAngle(double angle){
        wristPos = angle;
        wristPID.setSetpoint(angle);
    }

    public double getWristPos(){
        return wristAbsolute.getPosition();
    }

    public double getWristVelocity(){
        return wristAbsolute.getVelocity();
    }

    public double getIntakeVelocity(){
        return intakeEncoder.getVelocity();
    }

    public void setIntakeSpeed(double speed){
        intakeSet = speed;
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Angle Setpoint", Conversions.wristEncoderToDegrees(wristPos));
        SmartDashboard.putNumber("Wrist Absolute", Conversions.wristEncoderToDegrees(wristAbsolute.getPosition()));
        
        TrapezoidProfile wristProfile = new TrapezoidProfile(
            new Constraints(40, 8),
            new State(wristPos, 0),
            m_WristSetPoint
        );
        m_WristSetPoint = wristProfile.calculate(0.02);
        wristPID.setSetpoint(m_WristSetPoint.position);
        wristTilt.set(MathUtil.clamp(wristPID.calculate(wristAbsolute.getPosition()), -0.75, 0.75)-wristFF*Math.sin((RobotContainer.arm.getArmAngle()*(0.25/60)*2*Math.PI) + (wristAbsolute.getPosition()-0.5)*2*Math.PI));

        intake.set(intakeSet);


        /* Code to find FF
        SmartDashboard.putNumber("WristFF", testJoystick.getThrottle()*0.2);
        wristTilt.set(testJoystick.getThrottle()*0.2);
        if(testJoystick.getTrigger()){
            intake.set(1);
        }else if(testJoystick.getRawButton(2)){
            intake.set(-1);
        }else{
            intake.set(0);
        }
        */
        
    }
}
