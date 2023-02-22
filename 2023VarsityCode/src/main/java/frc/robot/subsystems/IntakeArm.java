package frc.robot.subsystems;

import frc.robot.Constants;
import java.util.Map;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase{

    private CANSparkMax armTilt1;
    private CANSparkMax armTilt2;
    private CANSparkMax armExtension;
    private WPI_TalonSRX wristTilt;

    private RelativeEncoder armTiltEncoder;
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
    private boolean brakeEnable = false;


    public IntakeArm() {

        /* Arm Tilt Motor Configs */
        armTilt1 = new CANSparkMax(Constants.IntakeArm.armTilt1CAN, MotorType.kBrushless);
        armTilt2 = new CANSparkMax(Constants.IntakeArm.armTilt2CAN, MotorType.kBrushless); 

        armTiltEncoder = armTilt1.getEncoder();
        armTiltEncoder.setPosition(0);
        armTilt2.follow(armTilt1, true);
        armTiltPID = armTilt1.getPIDController();
    
        armTiltPID.setP(0.1);
        armTiltPID.setI(0);
        armTiltPID.setD(0.003);
        armTiltPID.setFF(0.0002);
        armTiltPID.setOutputRange(-0.5, 0.5);
        armTilt1.setIdleMode(IdleMode.kBrake);
        armTilt2.setIdleMode(IdleMode.kBrake);
        armTiltPID.setSmartMotionMaxAccel(2 * Constants.IntakeArm.ArmTiltRatio, 0);

        //armTilt1.setSoftLimit(SoftLimitDirection.kForward, 0);
        //armTilt1.setSoftLimit(SoftLimitDirection.kReverse, 0);

        armTiltPID.setReference(0, ControlType.kPosition);


        /* Elevator Motor Config */
        armExtension = new CANSparkMax(Constants.IntakeArm.armExtensionCAN, MotorType.kBrushless);

        armExtensionEncoder = armExtension.getEncoder();
        armExtensionEncoder.setPosition(0);
        armExtendPID = armExtension.getPIDController();

        armExtendPID.setP(0.5);
        armExtendPID.setI(0);
        armExtendPID.setD(0.0004);
        armExtendPID.setFF(0.00017);
        armExtendPID.setOutputRange(-0.75, 0.2);
        armExtendPID.setSmartMotionMaxAccel(2*9.4,0);
        armExtension.setIdleMode(IdleMode.kBrake);

        //upperElevatorLimit = armExtension.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        //lowerElevataorLimit = armExtension.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        
        armExtendPID.setReference(0, ControlType.kPosition);


        /* Wrist Motor Config */
        wristTilt = new WPI_TalonSRX(Constants.IntakeArm.wristAngleCAN);

        /*/
        wristTilt.configForwardSoftLimitThreshold(0);
        wristTilt.configReverseSoftLimitThreshold(0);
        wristTilt.configForwardSoftLimitEnable(true);
        wristTilt.configReverseSoftLimitEnable(true);
        */

        wristTilt.setSelectedSensorPosition(0);
        wristTilt.configPeakOutputForward(0.5);
        wristTilt.configPeakOutputReverse(-0.5);
        wristTilt.config_kP(0, 0.12);
        wristTilt.configAllowableClosedloopError(0, 0, 0);

        wristTilt.set(TalonSRXControlMode.Position, 0);

        

        /* Intake Motor Config */
        intake = new CANSparkMax(Constants.IntakeArm.intakeCAN, MotorType.kBrushless);
        intakeEncoder = intake.getEncoder();
        intakeHoldPID = intake.getPIDController();

        intakeHoldPID.setP(0.8);

        intakeHoldPID.setReference(intakeEncoder.getPosition(), ControlType.kPosition);

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
                return new double[] {55.88,0,-826};
            case CUBE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {52.90,-1.07,1791};
            case AUTOCUBE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {-40.54,-3.04,-4517};
            case HUMANCUBE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {9.98,-11.98,8870};
            case HUMANCONE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {8.26,-13.83,8367};
            case HUMANSLIDE:
                intakeAction = IntakeAction.INTAKE;
                return new double[] {16.48,0,-2384};
            default:
                intakeAction = IntakeAction.HOLD;
                return new double[] {0,0,0};
        }

    }

    public Command runToPosition(Position pos){
        double[] armState = getArmState(pos);
        return new SequentialCommandGroup(
            //Shutoff intake
            Commands.runOnce(() -> intakeHoldPID.setReference(intakeEncoder.getPosition(), ControlType.kPosition)),

            //Disable Brake
            Commands.runOnce(() -> disableBrake()),

            //Elevator & wrist to 0
            Commands.run(() -> armExtendPID.setReference(0, ControlType.kPosition))
                .until(() -> Math.abs(armExtensionEncoder.getPosition()) < 2),
            //.alongWith(
                Commands.run(() -> wristTilt.set(TalonSRXControlMode.Position, 0))
                    .until(() -> Math.abs(wristTilt.getSelectedSensorPosition()) < 200),
            //),

            //Arm to angle
            Commands.run(() -> armTiltPID.setReference(armState[0], ControlType.kPosition))
                .until(() -> Math.abs(armTiltEncoder.getPosition() - armState[0]) < 2),
            
            //Elevator & wrist to position
            Commands.run(() -> armExtendPID.setReference(armState[1], ControlType.kPosition))
                .until(() -> Math.abs(armExtensionEncoder.getPosition() - armState[1]) < 2),
            //.alongWith(
                Commands.run(() -> wristTilt.set(TalonSRXControlMode.Position, armState[2]))
                    .until(() -> Math.abs(wristTilt.getSelectedSensorPosition() - armState[2]) < 200),
            //),

            //Activate Intake
            runIntake()

        );
    }

    private IntakeAction select(){
        return intakeAction;
    }

    private Command runIntake(){
        return new SelectCommand(
            Map.ofEntries(
                Map.entry(IntakeAction.HOLD, new InstantCommand(() -> intakeHoldPID.setReference(intakeEncoder.getPosition(), ControlType.kPosition))),
                Map.entry(IntakeAction.INTAKE, new InstantCommand(() -> intake.set(-1))),
                Map.entry(IntakeAction.PLACE, new InstantCommand(() -> intake.set(1)))
            ),
            this::select);
    }
    
    
    public double getArmAngle(){
        return armTiltEncoder.getPosition();
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
        HUMANSLIDE
    }

    public enum IntakeAction{
        HOLD,
        INTAKE,
        PLACE
    }

}
