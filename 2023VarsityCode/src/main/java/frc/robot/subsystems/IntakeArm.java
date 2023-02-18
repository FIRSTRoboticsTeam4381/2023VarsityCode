package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.annotation.SimpleObjectIdResolver;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {

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

    private double armAngle = 0;
    private double wristAngle = 0;
    private double extension = 0; 

    private Position position = Position.TRANSIT;
    private boolean brakeEnable = false;


    public IntakeArm() {

        armTilt1 = new CANSparkMax(Constants.IntakeArm.armTilt1CAN, MotorType.kBrushless);
        armTilt2 = new CANSparkMax(Constants.IntakeArm.armTilt2CAN, MotorType.kBrushless);
        armExtension = new CANSparkMax(Constants.IntakeArm.armExtensionCAN, MotorType.kBrushless);
        wristTilt = new WPI_TalonSRX(Constants.IntakeArm.wristAngleCAN);

        armTiltEncoder = armTilt1.getEncoder();
        armExtensionEncoder = armExtension.getEncoder();
        armTilt2.follow(armTilt1);
        armTiltPID = armTilt1.getPIDController();
        armExtendPID = armExtension.getPIDController();
    
        armTiltPID.setP(0.1);
        armTiltPID.setI(0);
        armTiltPID.setD(0.003);
        armTiltPID.setFF(0.0002);
        armTiltPID.setOutputRange(-0.3, 0.3);
        armTilt1.setIdleMode(IdleMode.kBrake);


        armExtendPID.setP(0.5);
        armExtendPID.setI(0);
        armExtendPID.setD(0.0004);
        armExtendPID.setFF(0.00017);
        armExtendPID.setOutputRange(-0.5, 0.5);
        armExtension.setIdleMode(IdleMode.kBrake);

        armTiltPID.setSmartMotionMaxAccel(2 * Constants.IntakeArm.ArmTiltRatio, 0);

        upperElevatorLimit = armExtension.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        lowerElevataorLimit = armExtension.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        armTilt1.setSoftLimit(SoftLimitDirection.kForward, 0);
        armTilt1.setSoftLimit(SoftLimitDirection.kReverse, 0);

        wristTilt.configForwardSoftLimitThreshold(0);
        wristTilt.configReverseSoftLimitThreshold(0);
        wristTilt.configForwardSoftLimitEnable(true);
        wristTilt.configReverseSoftLimitEnable(true);

        intake = new CANSparkMax(Constants.IntakeArm.intakeCAN, MotorType.kBrushless);

        armTilt1.set(0);
        armTilt2.set(0);
        armExtension.set(0);
        wristTilt.set(0);
        intake.set(0);
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

    public void setPosition(Position pos){
        position = pos;
    }

    public void goToPosition(Position pos){
        switch(pos){
            case TRANSIT:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
            case HIGHPLACE:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
            case MIDPLACE:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
            case UPCONE:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
            case TIPCONE:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
            case HUMANUPRIGHT:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
            case HUMANSLIDE:
                CommandScheduler.getInstance().schedule(runToPosition(0, 0, 0));
                break;
        }

    }

    public Command runToPosition(double wrist, double arm, double elevator){
        return new SequentialCommandGroup(
            //Elevator Down
            /*
            new FunctionalCommand(
                null, //init
                null, //execute
                null, //end
                null, //finish condition
                null //subsytem requirement
            ),
            */
            //Arm to angle
            new FunctionalCommand(
                () -> disableBrake(), //init
                () -> armTiltPID.setReference(arm, ControlType.kPosition, 0), //execute
                interrupted -> enableBrake(), //end
                () -> Math.abs(armTiltEncoder.getPosition() - arm) < 0.1, //finish condition
                this //subsytem requirement
            )
            //Elevator to Pos
            /*
            new FunctionalCommand(
                null, //init
                null, //execute
                null, //end
                null, //finish condition
                null //subsytem requirement
            )
            */
        );
    }
    
    /**TODO */
    public void setWristAngle(double angle){
        wristTilt.set(ControlMode.Position, angle);
    }

    /**TODO */
    public double getWwristAngle(){
        return wristTilt.getSelectedSensorPosition();
    }

    /**TODO */
    public void setArmAngle(double angle){}

    /**TODO */
    public double getArmAngle(){
        return armTiltEncoder.getPosition();
    }
    
    /**TODO */
    public void setElevatorExtend(double distance){}

    @Override
    public void periodic() {
        goToPosition(position);
    }


    public enum Position{
        TRANSIT,
        HIGHPLACE,
        MIDPLACE,
        UPCONE,
        TIPCONE,
        HUMANUPRIGHT,
        HUMANSLIDE
    }

}
