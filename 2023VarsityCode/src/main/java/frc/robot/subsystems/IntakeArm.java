package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeArm extends SubsystemBase{

    private CANifier leds;
    private double[] yellow = {0.2,1,0};
    private double[] purple = {0,1,1};
    private double[] set = {};

    private CANSparkMax armTilt1;
    private CANSparkMax armTilt2;
    private CANSparkMax armExtend;
    private WPI_TalonSRX wristTilt;

    private AbsoluteEncoder wristAbsolute;
    private PIDController wristPID;

    private RelativeEncoder armTilt1Encoder;
    private RelativeEncoder armTilt2Encoder;
    private RelativeEncoder armExtensionEncoder;
    private AbsoluteEncoder armPivotEncoder;

    private SparkMaxPIDController armTiltPID;
    private SparkMaxPIDController armExtendPID;

    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private SparkMaxPIDController intakeHoldPID;

    private double wristPos = 0.5;
    private double anglePos = 0;
    private double elevatePos = 0;

    private double wristOffset = 0;


    public IntakeArm() {

        /* Arm Tilt Motor Configs */
        armTilt1 = new CANSparkMax(Constants.IntakeArm.armTilt1CAN, MotorType.kBrushless);
        armTilt2 = new CANSparkMax(Constants.IntakeArm.armTilt2CAN, MotorType.kBrushless);
        armExtend = new CANSparkMax(Constants.IntakeArm.armExtensionCAN, MotorType.kBrushless);
        intake = new CANSparkMax(Constants.IntakeArm.intakeCAN, MotorType.kBrushless);
        intake.setIdleMode(IdleMode.kBrake);
        intake.setSmartCurrentLimit(40);

        wristTilt = new WPI_TalonSRX(Constants.IntakeArm.wristAngleCAN);

        armTilt1Encoder = armTilt1.getEncoder();
        armTilt2Encoder = armTilt2.getEncoder();
        armPivotEncoder = armTilt1.getAbsoluteEncoder(Type.kDutyCycle);

        armTilt1Encoder.setPosition((armPivotEncoder.getPosition()-0.5)*240);
        armTilt2Encoder.setPosition(0);

        armExtensionEncoder = armExtend.getEncoder();

        //armTilt2.follow(armTilt1,true);
        armTilt2.setIdleMode(IdleMode.kBrake);
        armTilt2.set(0);

        armTiltPID = armTilt1.getPIDController();
        armTiltPID.setP(0.1);
        armTiltPID.setI(0);
        armTiltPID.setD(0.0015);
        armTiltPID.setFF(0.0002);
        armTiltPID.setOutputRange(-1, 1);
        armTilt1.setIdleMode(IdleMode.kBrake);

        armTiltPID.setSmartMotionMaxAccel(2 * Constants.IntakeArm.ArmTiltRatio, 0);

        armExtendPID = armExtend.getPIDController();
        armExtendPID.setP(0.4);
        armExtendPID.setI(0);
        armExtendPID.setD(0.0004);
        armExtendPID.setFF(0.00017);
        armExtendPID.setOutputRange(-0.75, 0.4);
        armExtendPID.setSmartMotionMaxAccel(2*9.4,0);
        armExtend.setIdleMode(IdleMode.kBrake);

        wristTilt.configFactoryDefault();
        wristTilt.setNeutralMode(NeutralMode.Brake);
        wristTilt.configPeakOutputForward(0.6);
        wristTilt.configPeakOutputReverse(-0.6);
        
        wristAbsolute = intake.getAbsoluteEncoder(Type.kDutyCycle);
        wristPID = new PIDController(0.1, 0, 0);
        
        intakeEncoder = intake.getEncoder();
        intakeEncoder.setPosition(0);
        intakeHoldPID = intake.getPIDController();
        intakeHoldPID.setP(0.5);
        intakeHoldPID.setReference(0, ControlType.kPosition);

        leds = new CANifier(49);
    }

    public double getArmAngle(){
        return armTilt1Encoder.getPosition(); //*armConversionfactor gear ratios and stuff
    }

    public double getArmVelocity(){
        return armTilt1Encoder.getVelocity(); //*arm velocity conversion
    }

    public double getElevateHeight(){
        return armExtensionEncoder.getPosition();
    }

    public double getElevateVelocity(){
        return armExtensionEncoder.getVelocity();
    }

    public void setArmAngle(double angle){
        anglePos = angle;
        //armTiltPID.setReference(angle, ControlType.kPosition);
    }

    public void setElevator(double height){
        elevatePos = height;
    }

    public void setWristAngle(double angle){
        wristPos = angle;
        wristPID.setSetpoint(angle);
    }

    private TrapezoidProfile.State m_ArmSetPoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_ElevatorSetPoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_WristSetPoint = new TrapezoidProfile.State();


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Tilt Encoder", armTilt1Encoder.getPosition());
        SmartDashboard.putNumber("Arm Tilt Degrees", armTilt1Encoder.getPosition()*(0.25/60)*360);
        SmartDashboard.putNumber("Arm Absolute", armPivotEncoder.getPosition());
        SmartDashboard.putNumber("Arm Extend Encoder", armExtensionEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Absolute", wristAbsolute.getPosition());
        
        SmartDashboard.putNumber("Arm Angle Setpoint", anglePos);
        SmartDashboard.putNumber("Arm Extend Setpoint", elevatePos);
        SmartDashboard.putNumber("Wrist Angle Setpoint", wristPos);

        TrapezoidProfile armProfile = new TrapezoidProfile(
            new Constraints(3000, 500),//Could use a little less accel
            new State(anglePos, 0),
            m_ArmSetPoint
        );
        m_ArmSetPoint = armProfile.calculate(0.02);
        armTiltPID.setReference(m_ArmSetPoint.position, ControlType.kPosition);

        TrapezoidProfile elevateProfile = new TrapezoidProfile(
            new Constraints(1000, 100),//Little more accel, higher power/velocity
            new State(elevatePos, 0),
            m_ElevatorSetPoint
        );
        m_ElevatorSetPoint = elevateProfile.calculate(0.02);
        armExtendPID.setReference(m_ElevatorSetPoint.position, ControlType.kPosition);


        /* Untested, need to figure out incorporating feed forward
        TrapezoidProfile wristProfile = new TrapezoidProfile(
            new Constraints(10, 1),
            new State(wristPos, 0),
            m_WristSetPoint
        );
        m_WristSetPoint = wristProfile.calculate(0.02);
        wristTilt.set(-0.0625*Math.sin(armTilt1Encoder.getPosition()*(0.25/60)*2*Math.PI));
        //FF 0.0625
        */
    }
}
