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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeArm extends SubsystemBase{

   // private CANifier leds;
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

    private Position position = Position.TRANSIT;
    private IntakeAction intakeAction = IntakeAction.HOLD;
    private double intakeHoldPos = 0;
    private double intakePlaceNum = 0;
    private boolean brakeEnable = false;
    public boolean LOCKOUT = false;
    private boolean atSpeed = false;

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

        //Enable brake
        enableBrake();

        //leds = new CANifier(49);
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
            case TRANSIT://GOOD
                intakeAction = IntakeAction.HOLD;
                return new double[] {0,0,0};
            case HIGHPLACE://GOOD - Move up a tad because it hits pylon sometimes
                intakeAction = IntakeAction.PLACE;
                return new double[] {29.12,-32.0,6583};
            case MIDPLACE://GOOD - Move up a tad because it hits pylon sometimes
                intakeAction = IntakeAction.PLACE;
                return new double[] {30.85,-15.64,5950};
            case UPCONE://GOOD - Check again
                intakeAction = IntakeAction.INTAKE;
                return new double[] {62,0,-2000};
            case CUBE://GOOD
                intakeAction = IntakeAction.INTAKE;
                return new double[] {56.90,-1.07,2091};
            case AUTOCUBE://GOOD
                intakeAction = IntakeAction.INTAKE;
                return new double[] {-54.69,0,-3259};
            case HUMANCUBE://GOOD
                intakeAction = IntakeAction.INTAKE;
                return new double[] {9.98,-11.98,8870};
            case HUMANCONE://GOOD
                intakeAction = IntakeAction.INTAKE;
                return new double[] {12.42,-12.5,6500};
            case HUMANSLIDE://GOOD
                intakeAction = IntakeAction.INTAKE;
                return new double[] {-48.16,0,4631};
            case HYBRID://GOOD
                intakeAction = IntakeAction.PLACE;
                return new double[] {55.88,0,-826};
            case TIPCONE://GOOD
                intakeAction = IntakeAction.INTAKE;
                return new double[] {-62.09,-2.21,329};
            case SHOOTMIDCUBE://GOOD
                intakeAction = IntakeAction.SHOOT;
                return new double[] {0,0,4000};
            case SHOOTHIGHCUBE://GOOD
                intakeAction = IntakeAction.SHOOT;
                return new double[] {32,-15,-1000};
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
    public double getIntakeVelocity(){
        if(!atSpeed){
            return 101;
        }else{
            return intakeEncoder.getVelocity();
        }
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
        SHOOTMIDCUBE,
        SHOOTHIGHCUBE
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
        /*
        set = (RobotContainer.stationSelector.getType() == Type.CUBE)?purple:yellow;
        leds.setLEDOutput(set[0], LEDChannel.LEDChannelA);
        leds.setLEDOutput(set[1], LEDChannel.LEDChannelB);
        leds.setLEDOutput(set[2], LEDChannel.LEDChannelC);
        */

        SmartDashboard.putNumber("intake encoder velocity", intakeEncoder.getVelocity());
        SmartDashboard.putNumber("intake Encoder", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle encoder",armTilt1Encoder.getPosition()/1.474);
        SmartDashboard.putNumber("Arm Extend encoder", armExtensionEncoder.getPosition());
        SmartDashboard.putNumber("Arm Wrist encoder", wristTilt.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm angle setpoint", getArmState(position)[0]);
        SmartDashboard.putNumber("Arm extend setpoint", getArmState(position)[1]);
        SmartDashboard.putNumber("Arm wrist setpoint", getArmState(position)[2]/3.68);

        if(position == Position.TRANSIT){
            if(Math.abs(armExtensionEncoder.getPosition()) < 15){
                armTiltPID.setReference(1.474*getArmState(position)[0], ControlType.kPosition);
            }else{
                armTiltPID.setReference(1.474*getArmState(Position.HIGHPLACE)[0], ControlType.kPosition);
            }
        }else{
            armTiltPID.setReference(1.474*getArmState(position)[0], ControlType.kPosition);
        }

        if(Math.abs(armTilt1Encoder.getPosition()/1.474 - getArmState(position)[0]) < 2){
            armExtendPID.setReference(getArmState(position)[1], ControlType.kPosition);
            if(position == Position.HIGHPLACE){
                if(armExtensionEncoder.getPosition() < -12){
                    wristTilt.set(TalonSRXControlMode.Position, getArmState(position)[2]/3.68); //Wrist down
                }else{
                    wristTilt.set(TalonSRXControlMode.Position, 0);
                }
            }else{
                wristTilt.set(TalonSRXControlMode.Position, getArmState(position)[2]/3.68); //Wrist down
            }
        }else{
            armExtendPID.setReference(0, ControlType.kPosition);
            wristTilt.set(TalonSRXControlMode.Position, 0);
        }

        if(Math.abs(armExtensionEncoder.getPosition() - getArmState(position)[1]) < 1 && Math.abs(armTilt1Encoder.getPosition() - getArmState(position)[0]*1.474) < 10){
            switch(intakeAction){
                case HOLD:
                    if(RobotContainer.stationSelector.getType() == Type.CUBE){
                        intake.set(-0.05);
                        intakeHoldPos = intakeEncoder.getPosition();
                    }else{
                        intakeHoldPID.setReference(intakeHoldPos, ControlType.kPosition);
                    }
                    intakePlaceNum = intakeEncoder.getPosition();
                    atSpeed = false;
                    break;
                case INTAKE:
                    intakeHoldPos = intakeEncoder.getPosition()-1;
                    intake.set(-1);
                    if(Math.abs(intakeEncoder.getVelocity()) > 3500){
                        atSpeed = true;
                    }
                    break;
                case SHOOT:
                    intakeHoldPos = intakeEncoder.getPosition()+0.01;
                    if(Math.abs(wristTilt.getSelectedSensorPosition() - (getArmState(position)[2]/3.68)) < 300){
                        intake.set(0.75);
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
                            intakeHoldPos = intakeEncoder.getPosition()+2;
                            intake.set(1);
                            break;
                    }
                }
        }

        LOCKOUT = armExtensionEncoder.getPosition() < -3 && (position == Position.HIGHPLACE || position == Position.MIDPLACE);

    }
}
