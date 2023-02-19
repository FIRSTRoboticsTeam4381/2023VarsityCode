// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.cameraserver.CameraServer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CANSparkMax armTilt1;
  private CANSparkMax armTilt2;
  private CANSparkMax armExtend;
  private WPI_TalonSRX wristTilt;
  private CANSparkMax intake;

  private RelativeEncoder intakeEncoder;
  private SparkMaxPIDController intakeHoldPID;

  private Servo brake;

  private RelativeEncoder armTilt1Encoder;
  private RelativeEncoder armTilt2Encoder;
  private AbsoluteEncoder armTiltAbsoluteEncoder;
  private RelativeEncoder armExtensionEncoder;

  private SparkMaxPIDController armTiltPID;
  private SparkMaxPIDController armExtendPID;
  private boolean brakeEnable = true;
  private final Joystick testingController = new Joystick(3);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
   
    CameraServer.startAutomaticCapture();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

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

    armTiltAbsoluteEncoder = armTilt1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    armExtensionEncoder = armExtend.getEncoder();

    armTilt2.follow(armTilt1,true);

    brake = new Servo(1);

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
    armExtendPID.setOutputRange(-0.75, 0.2);
    armExtendPID.setSmartMotionMaxAccel(2*9.4,0);
    armExtend.setIdleMode(IdleMode.kBrake);

    wristTilt.configFactoryDefault();
    wristTilt.setNeutralMode(NeutralMode.Brake);
    wristTilt.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    wristTilt.setSelectedSensorPosition(0);
    wristTilt.configPeakOutputForward(0.5);
    wristTilt.configPeakOutputReverse(-0.5);
    wristTilt.config_kP(0, 0.1);
    wristTilt.configAllowableClosedloopError(0, 0, 0);
    
    intakeEncoder = intake.getEncoder();
    intakeHoldPID = intake.getPIDController();
    intakeHoldPID.setP(0.5);

    //armTilt1.setSoftLimit(SoftLimitDirection.kForward, 0);
    //armTilt1.setSoftLimit(SoftLimitDirection.kReverse, 0);
    

    armTilt1.set(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putString("Station", RobotContainer.stationSelector.getStation());
    SmartDashboard.putString("Type", RobotContainer.stationSelector.getType());
    SmartDashboard.putString("KeyStroke", RobotContainer.stationSelector.getStroke());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    SmartDashboard.putNumber("Arm angle 1", armTilt1Encoder.getPosition());
    SmartDashboard.putNumber("Arm angle 2", armTilt2Encoder.getPosition());
    SmartDashboard.putNumber("Arm angle Absolute", armTiltAbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("Wrist angle", wristTilt.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Extension", armExtensionEncoder.getPosition());

    if(testingController.getRawButton(3)){
      double pos = 22.618;
      armTiltPID.setReference(pos, ControlType.kPosition);
      if(Math.abs(armTilt1Encoder.getPosition() - pos) < 2){
        armExtendPID.setReference(-17.238, ControlType.kPosition);
        wristTilt.set(TalonSRXControlMode.Position, 6519); //Wrist down
      }else{
        armExtendPID.setReference(0, ControlType.kPosition);
        wristTilt.set(TalonSRXControlMode.Position, 0);
      }
    }else if(testingController.getRawButton(2)){
      wristTilt.set(TalonSRXControlMode.Position, 581);
      armTiltPID.setReference(53.25, ControlType.kPosition);
      armExtendPID.setReference(0, ControlType.kPosition);
    }else{
      armTiltPID.setReference(0, ControlType.kPosition);
      armExtendPID.setReference(0, ControlType.kPosition);
      wristTilt.set(TalonSRXControlMode.Position, 0);
    }

    if(testingController.getRawButton(1)){
      intake.set(1); //Fire out
    }else if(testingController.getRawButton(2)){
      intake.set(-1); //Pull in
    }else{
      intakeHoldPID.setReference(intakeEncoder.getPosition(), ControlType.kPosition);
    }

    brake.set((testingController.getThrottle()+1)/2);

  }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
