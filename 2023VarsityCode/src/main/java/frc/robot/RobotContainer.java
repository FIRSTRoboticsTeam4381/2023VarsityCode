// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ArmPositions.Position;
import frc.robot.ArmPositions.Type;
import frc.robot.autos.Autos;
import frc.robot.commands.ArmDefault;
import frc.robot.commands.ElevatorDefault;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WristDefault;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandPS4Controller driveController = new CommandPS4Controller(0);
  private final CommandPS4Controller specialsController = new CommandPS4Controller(1);

  /* Driver Buttons */
  private final Trigger zeroSwerve = driveController.options();
  
  /* Subsystems */
  public static final Swerve s_Swerve = new Swerve();
  public static final ArmAngleSubsystem arm = new ArmAngleSubsystem();
  public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static final WristSubsystem wrist = new WristSubsystem();
  public static final IntakeCommands armCommand = new IntakeCommands(arm, elevator, wrist);
  public static final LEDS leds = new LEDS();

  public static final StationSelector stationSelector = new StationSelector(Position.HIGHPLACE, Type.CONE);

  /* Auto Chooser */
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driveController, true));
    arm.setDefaultCommand(new ArmDefault(arm));
    elevator.setDefaultCommand(new ElevatorDefault(elevator));
    wrist.setDefaultCommand(new WristDefault(wrist));

    // Configure the button bindings
    configureButtonBindings();
    
    //Add autonoumous options to chooser
    m_AutoChooser.setDefaultOption("None", Autos.none());
    m_AutoChooser.addOption("Three Piece Grab", Autos.threePiece());
    m_AutoChooser.addOption("ConePark", Autos.coneBalance());
    m_AutoChooser.addOption("Three Piece Place", Autos.threePiecePlace());
    m_AutoChooser.addOption("Two Piece Balance", Autos.twoPieceBalance());
    m_AutoChooser.addOption("Three Piece HIGH", Autos.threePieceHIGH());
    m_AutoChooser.addOption("CONE PARK NO OVER", Autos.coneParkNoMobility());
    m_AutoChooser.addOption("Bump Auto", Autos.bumpPath());
    SmartDashboard.putData(m_AutoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    /* Swerve Reset Button */
    zeroSwerve
      .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0))
      .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));     

    //driveController.cross().onTrue(new InstantCommand(() -> s_Swerve.addVisionMeasurement()));

    /* Ash binding
    driveController.R1().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(armCommand.placeElevator(ArmPositions.getArmState(stationSelector.getPos())))));
    */

    /* Ryan Bindings
     */
    //driveController.axisGreaterThan(4, 0.5).onTrue(new InstantCommand(() -> arm.resetArm()));
    /*
    driveController.cross().onTrue(
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(
        armCommand.placeElevator(ArmPositions.getArmState(stationSelector.getPos())))));

    driveController.R1().onTrue(
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(
        armCommand.prePlace(ArmPositions.getArmState((stationSelector.getType() == Type.CONE)?Position.PREPLACECONE:Position.PREPLACECUBE)))));
    */

    driveController.R1().onTrue(
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(
        armCommand.preplaceElevator(ArmPositions.getArmState(stationSelector.getPos()))))
    );

    driveController.cross().onTrue(
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(
        armCommand.placeAndReturn(ArmPositions.getArmState(stationSelector.getPos()))))
    );

        /*
    driveController.square()
      .onTrue(new InstantCommand(() -> stationSelector.setType(Type.CONE))
        .andThen(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.HUMANCONE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 100 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(-0.1)))));
    
    driveController.circle()
      .onTrue(new InstantCommand(() -> stationSelector.setType(Type.CUBE))
      .andThen(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.HUMANCUBE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 100 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(-0.1)))));
*/
    
    driveController.touchpad().or(specialsController.touchpad()).onTrue(armCommand.returnToHome(0));

    specialsController.R1().onTrue(new InstantCommand(() -> stationSelector.setType(Type.CUBE)));
    specialsController.L1().onTrue(new InstantCommand(() -> stationSelector.setType(Type.CONE)));

    specialsController.povUp().onTrue(new InstantCommand(() -> stationSelector.setPos(Position.HIGHPLACE)));
    specialsController.povDown().onTrue(new InstantCommand(() -> stationSelector.setPos(Position.HYBRID)));
    specialsController.povLeft()
      .or(specialsController.povRight())
      .onTrue(new InstantCommand(() -> stationSelector.setPos(Position.MIDPLACE)));


    specialsController.R1()
      .and(specialsController.triangle())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.HUMANSLIDE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(-0.1))));

    specialsController.R1()
      .and(specialsController.circle())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.AUTOCUBE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(-0.1))));

    specialsController.R1()
      .and(specialsController.square())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.HUMANCUBE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(-0.1))));

    specialsController.R1()
      .and(specialsController.cross())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.CUBE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(-0.1))));

    specialsController.L1()
      .and(specialsController.triangle())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.HUMANSLIDE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(0))));

    specialsController.L1()
      .and(specialsController.circle())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.TIPCONE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(0))));

    specialsController.L1()
      .and(specialsController.square())
      .onTrue(armCommand.intakePosition(ArmPositions.getArmState(ArmPositions.Position.HUMANCONE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(0))));

    specialsController.L1()
      .and(specialsController.cross())
      .onTrue(armCommand.uprightConeIntake(ArmPositions.getArmState(ArmPositions.Position.UPCONE))
      .andThen(new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) < 200 || specialsController.touchpad().getAsBoolean())
      .andThen(armCommand.returnToHome(0
      ))));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoChooser.getSelected();
  }
}