package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeArm extends SubsystemBase{

    private CANSparkMax armTilt1;
    private CANSparkMax armTilt2;
    private CANSparkMax armExtend;
    private WPI_TalonSRX wristTilt;

    private RelativeEncoder armTilt1Encoder;
    private RelativeEncoder armTilt2Encoder;
    private RelativeEncoder armExtensionEncoder;

    private SparkMaxPIDController armTiltPID;
    private SparkMaxPIDController armExtendPID;

    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private SparkMaxPIDController intakeHoldPID;

    private Position position = Position.TRANSIT;
    private IntakeAction intakeAction = IntakeAction.HOLD;
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
        armExtendPID.setP(0.4);
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
        intakeEncoder.setPosition(0);
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
                return new double[] {25.83,-30.90,6357};
            case MIDPLACE:
                intakeAction = IntakeAction.PLACE;
                return new double[] {26.47,-18.38,8200};
            case UPCONE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {55.88,0,-826};
            case CUBE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {52.90,-1.07,1791};
            case AUTOCUBE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {-54.69,0,-3259};
            case HUMANCUBE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {9.98,-11.98,8870};
            case HUMANCONE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {8.26,-13.83,8367};
            case HUMANSLIDE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {40.28,0,-3469};
            case HYBRID:
                intakeAction = IntakeAction.PLACE;
                return new double[] {55.88,0,-826};
            case TIPCONE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {-62.09,-2.21,329};
            case SHOOTCUBE:
                intakeAction = IntakeAction.SHOOT;
                return new double[] {0,0,4000};
            default:
                intakeAction = IntakeAction.HOLD;
                return new double[] {0,0,0};
        }

    }
    
    public double getArmAngle(){
        return armTilt1Encoder.getPosition();
    }

    public double getIntakeEncoder(){
        return intakeEncoder.getPosition();
    }
    public double intakePlacePos(){
        return intakePlaceNum;
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
        HYBRID,
        TIPCONE,
        SHOOTCUBE
    }

    public enum IntakeAction{
        HOLD,
        INTAKE,
        PLACE,
        SHOOT
    }

    public enum Type{
        CUBE,
        CONE
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm State", getArmState(position)[0]);

        SmartDashboard.putNumber("intakePlace", intakePlaceNum);
        SmartDashboard.putNumber("intake Encoder", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle encoder", armTilt1Encoder.getPosition());
        SmartDashboard.putNumber("Arm Extend encoder", armExtensionEncoder.getPosition());
        SmartDashboard.putNumber("Arm Wrist encoder", wristTilt.getSelectedSensorPosition());


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
                    intakeHoldPos = intakeEncoder.getPosition()-0.04;
                    intake.set(-1);
                    break;
                case SHOOT:
                    intakeHoldPos = intakeEncoder.getPosition()+0.01;
                    if(Math.abs(wristTilt.getSelectedSensorPosition() - getArmState(position)[2]) < 500){
                        intake.set(1);
                    }else{
                        intake.set(0);
                    }
                    break;
                case PLACE:
                    switch(RobotContainer.stationSelector.getType()){
                        case CUBE:
                            intakeHoldPos = intakeEncoder.getPosition();
                            intake.set(0.25);
                            break;
                        case CONE:
                            intakeHoldPos = intakeEncoder.getPosition()+0.01;
                            intake.set(1);
                            break;
                    }
                }
        }

        LOCKOUT = armExtensionEncoder.getPosition() < -3 && (position == Position.HIGHPLACE || position == Position.MIDPLACE);

    }
}
