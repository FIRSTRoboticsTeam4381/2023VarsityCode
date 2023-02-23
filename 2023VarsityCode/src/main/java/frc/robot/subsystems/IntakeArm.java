package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase{

    private CANSparkMax armTilt1;
    private CANSparkMax armTilt2;
    private CANSparkMax armExtend;
    private WPI_TalonSRX wristTilt;

    private RelativeEncoder armTilt1Encoder;
    private RelativeEncoder armTilt2Encoder;
    private RelativeEncoder armExtensionEncoder;

    private SparkMaxLimitSwitch upperElevatorLimit;
    private SparkMaxLimitSwitch lowerElevataorLimit;

    private SparkMaxPIDController armTiltPID;
    private SparkMaxPIDController armExtendPID;
    private ArmFeedforward wristFF;
    private ArmFeedforward armFF;
    private ElevatorFeedforward extendFF;

    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private SparkMaxPIDController intakeHoldPID;

    private double armAngle = 0;
    private double wristAngle = 0;
    private double extension = 0; 

    private Position position = Position.TRANSIT;
    private IntakeAction intakeAction = IntakeAction.HOLD;
    private Type current = Type.CUBE;
    private double intakeHoldPos = 0;
    private double intakePlaceNum = 0;
    private boolean brakeEnable = false;
    public boolean LOCKOUT = false;

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

        armTilt1Encoder.setPosition(0);
        armTilt2Encoder.setPosition(0);

        armExtensionEncoder = armExtend.getEncoder();

        //armTilt2.follow(armTilt1,true);
        armTilt2.setIdleMode(IdleMode.kBrake);
        armTilt2.set(0);

        armTiltPID = armTilt1.getPIDController();
        armTiltPID.setP(0.1);
        armTiltPID.setI(0);
        armTiltPID.setD(0.003);
        armTiltPID.setFF(0.0002);
        armTiltPID.setOutputRange(-0.5, 0.5);
        armTilt1.setIdleMode(IdleMode.kBrake);

        armTiltPID.setSmartMotionMaxAccel(2 * Constants.IntakeArm.ArmTiltRatio, 0);

        armExtendPID = armExtend.getPIDController();
        armExtendPID.setP(0.5);
        armExtendPID.setI(0);
        armExtendPID.setD(0.0004);
        armExtendPID.setFF(0.00017);
        armExtendPID.setOutputRange(-0.75, 0.4);
        armExtendPID.setSmartMotionMaxAccel(2*9.4,0);
        armExtend.setIdleMode(IdleMode.kBrake);

        wristTilt.configFactoryDefault();
        wristTilt.setNeutralMode(NeutralMode.Brake);
        wristTilt.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        wristTilt.setSelectedSensorPosition(0);
        wristTilt.configPeakOutputForward(0.5);
        wristTilt.configPeakOutputReverse(-0.5);
        wristTilt.config_kP(0, 0.12);
        wristTilt.configAllowableClosedloopError(0, 0, 0);
        
        intakeEncoder = intake.getEncoder();
        intakeHoldPID = intake.getPIDController();
        intakeHoldPID.setP(0.5);
        intakeHoldPID.setReference(0, ControlType.kPosition);

        //Enable brake
        enableBrake();
    }

    private void enableBrake(){
        //Set brake
        brakeEnable = true;
    }
    private void disableBrake(){
        //Set brake
        brakeEnable = false;
    }

    public void setState(Position pos){
        position = pos;
    }

    public double[] getArmState(Position pos){
        switch(pos){
            case TRANSIT:
                intakeAction = IntakeAction.HOLD;
                return new double[] {0,0,0};
            case HIGHPLACE:
                intakeAction = IntakeAction.PLACE;
                return new double[] {23.88,-32.09,5765};
            case MIDPLACE:
                intakeAction = IntakeAction.PLACE;
                return new double[] {23.02,-18.33,6894};
            case UPCONE:
                intakeAction = IntakeAction.INTAKE;
                current = Type.CONE;
                return new double[] {55.88,0,-826};
            case CUBE:
                intakeAction = IntakeAction.INTAKE;
                current = Type.CUBE;
                return new double[] {52.90,-1.07,1791};
            case AUTOCUBE:
                intakeAction = IntakeAction.INTAKE;
                current = Type.CUBE;
                return new double[] {-40.54,-3.04,-4517};
            case HUMANCUBE:
                intakeAction = IntakeAction.INTAKE;
                current = Type.CUBE;
                return new double[] {9.98,-11.98,8870};
            case HUMANCONE:
                intakeAction = IntakeAction.INTAKE;
                current = Type.CONE;
                return new double[] {8.26,-13.83,8367};
            case HUMANSLIDE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {16.48,0,-2384};
            case HYBRID:
                intakeAction = IntakeAction.PLACE;
                return new double[] {55.88,0,-826};
            default:
                intakeAction = IntakeAction.HOLD;
                return new double[] {0,0,0};
        }

    }
    
    public double getArmAngle(){
        return armTilt1Encoder.getPosition();
    }

    public boolean placed(){
        return intakeAction == IntakeAction.PLACE && intakeEncoder.getPosition() > intakeHoldPos+3;
    }

    public enum Position{
        TRANSIT,
        HIGHPLACE,
        MIDPLACE,
        UPCONE,
        CUBE,
        AUTOCUBE,
        HUMANCUBE,
        HUMANCONE,
        HUMANSLIDE,
        HYBRID
    }

    public enum IntakeAction{
        HOLD,
        INTAKE,
        PLACE
    }

    public enum Type{
        CUBE,
        CONE
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm State", getArmState(position)[0]);

        if(position == Position.TRANSIT){
            if(Math.abs(armExtensionEncoder.getPosition()) < 15){
                armTiltPID.setReference(getArmState(position)[0], ControlType.kPosition);
            }else{
                armTiltPID.setReference(getArmState(Position.HIGHPLACE)[0], ControlType.kPosition);
            }
        }else{
            armTiltPID.setReference(getArmState(position)[0], ControlType.kPosition);
        }

        if(Math.abs(armTilt1Encoder.getPosition() - getArmState(position)[0]) < 2){
            armExtendPID.setReference(getArmState(position)[1], ControlType.kPosition);
            wristTilt.set(TalonSRXControlMode.Position, getArmState(position)[2]); //Wrist down
        }else{
            armExtendPID.setReference(0, ControlType.kPosition);
            wristTilt.set(TalonSRXControlMode.Position, 0);
        }

        if((Math.abs(armExtensionEncoder.getPosition() - getArmState(position)[1]) < 1)){
            switch(intakeAction){
                case HOLD:
                    intakeHoldPID.setReference(intakeHoldPos, ControlType.kPosition);
                    intakePlaceNum = intakeHoldPos;
                    break;
                case INTAKE:
                    intakeHoldPos = intakeEncoder.getPosition()-0.01;
                    intake.set(-1);
                    break;
                case PLACE:
                    switch(current){
                        case CUBE:
                            intakeHoldPos = intakeEncoder.getPosition();
                            intake.set(0.1);
                            break;
                        case CONE:
                            intakeHoldPos = intakeEncoder.getPosition()+0.01;
                            intake.set(1);
                    }
                /*
                    if(RobotContainer.stationSelector.getType().equals("Cone")){
                        intakeHoldPos = intakeEncoder.getPosition()+0.01;
                        intake.set(1);
                    }else if(RobotContainer.stationSelector.getType().equals("Cube")){
                        intakeHoldPos = intakeEncoder.getPosition();
                        intake.set(0.1);
                    }else{
                        intakeHoldPos = intakeEncoder.getPosition();
                        intake.set(0.5);
                    }                    
                    break;
                */
                }
        }

        LOCKOUT = armExtensionEncoder.getPosition() < -3 && (position == Position.HIGHPLACE || position == Position.MIDPLACE);

    }
}
