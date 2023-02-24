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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autos;
import frc.robot.autos.StationSelector;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm.Position;
import frc.robot.subsystems.IntakeArm.Type;
import frc.robot.subsystems.Swerve;

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

  /* Station Selector Buttons */
  public static StationSelector stationSelector = new StationSelector(Position.HIGHPLACE, Type.CONE);
  private final Trigger specialsLeftDpad = specialsController.povLeft();
  private final Trigger specialsRightDpad = specialsController.povRight();
  private final Trigger specialsTopDpad = specialsController.povUp();
  private final Trigger specialsBottomDpad = specialsController.povDown();
  
  /* Subsystems */
  public static final Swerve s_Swerve = new Swerve();
  public static final IntakeArm arm = new IntakeArm();

  /* Auto Chooser */
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driveController, true));

    // Configure the button bindings
    configureButtonBindings();
    
    //Add autonoumous options to chooser
    m_AutoChooser.setDefaultOption("None", Autos.none());
    m_AutoChooser.addOption("SingleConeAuto", Autos.singleCone());
    m_AutoChooser.addOption("PathPlanner Test", Autos.exampleAuto());
    m_AutoChooser.addOption("Micheals Eyes Worst Nightmare", Autos.blindMike());
    m_AutoChooser.addOption("Balance", Autos.balanceCommad());
    m_AutoChooser.addOption("Three Piece", Autos.threePiece());
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

    driveController.R1()
      .onTrue(Commands.run(() -> arm.setState(stationSelector.getPos()))
      .until(() -> arm.getIntakeEncoder() > arm.intakePlacePos()+3)
      .andThen(Commands.run(() -> arm.setState(Position.TRANSIT))
      .until(() -> Math.abs(arm.getArmAngle()) < 5)));

    //driveController.L1().onTrue(lineup());

    specialsController.R1().onTrue(new InstantCommand(() -> stationSelector.setType(Type.CUBE)));
    specialsController.L1().onTrue(new InstantCommand(() -> stationSelector.setType(Type.CONE)));

    specialsTopDpad.onTrue(new InstantCommand(() -> stationSelector.setPos(Position.HIGHPLACE)));
    specialsLeftDpad.onTrue(new InstantCommand(() -> stationSelector.setPos(Position.MIDPLACE)));
    specialsRightDpad.onTrue(new InstantCommand(() -> stationSelector.setPos(Position.MIDPLACE)));
    specialsBottomDpad.onTrue(new InstantCommand(() -> stationSelector.setPos(Position.HYBRID)));

    specialsController.R1()
      .and(specialsController.triangle())
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANSLIDE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.R1()
      .and(specialsController.circle())
      .onTrue(new InstantCommand(() -> arm.setState(Position.AUTOCUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.R1()
      .and(specialsController.square())
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANCUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.R1()
      .and(specialsController.cross())
      .onTrue(new InstantCommand(() -> arm.setState(Position.CUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.L1()
      .and(specialsController.triangle())
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANSLIDE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.L1()
      .and(specialsController.circle())
      .onTrue(new InstantCommand(() -> arm.setState(Position.TIPCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.L1()
      .and(specialsController.square())
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.L1()
      .and(specialsController.cross())
      .onTrue(new InstantCommand(() -> arm.setState(Position.UPCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));


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
