package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private RelativeEncoder armTilt1Encoder;
    private RelativeEncoder armTilt2Encoder;
    private RelativeEncoder armExtensionEncoder;
    private RelativeEncoder armPivotEncoder;

    private SparkMaxPIDController armTiltPID;
    private SparkMaxPIDController armExtendPID;

    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private SparkMaxPIDController intakeHoldPID;

    private double wristPos = 0;
    private double anglePos = 0;
    private double elevatePos = 0;

    private CommandPS4Controller testingPs4Controller = new CommandPS4Controller(1);

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
        //armPivotEncoder = armTilt1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        armTilt1Encoder.setPosition(0);
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
        wristTilt.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        wristTilt.setSelectedSensorPosition(0);
        wristTilt.configPeakOutputForward(0.6);
        wristTilt.configPeakOutputReverse(-0.6);
        wristTilt.config_kP(0, 1);
        wristTilt.config_kD(0, 0);
        wristTilt.config_kF(0, 0);
        wristTilt.configAllowableClosedloopError(0, 0, 0);
        
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

    public void setArmAngle(double angle){
        armTiltPID.setReference(angle// divided by armConversionFactor
        , ControlType.kPosition);
    }



    @Override
    public void periodic(){

        

    }
}
