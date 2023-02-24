// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeArm.Position;

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
  public static StationSelector
   stationSelector = new StationSelector(DriverStation.getAlliance());;
  private final Trigger specialsLeftDpad = specialsController.povLeft();
  private final Trigger specialsRightDpad = specialsController.povRight();
  private final Trigger specialsTopDpad = specialsController.povUp();
  private final Trigger specialsBottomDpad = specialsController.povDown();
  
  private final CommandJoystick testingController = new CommandJoystick(3);
  
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

    
    /* Station Selector Commands */
    specialsLeftDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("L")));
    specialsRightDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("R")));
    specialsTopDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("T")));
    specialsBottomDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("B")));


    /* Auto Place Command 
    driveController.R1().and(driveController.L1()).onTrue(
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(
        Autos.followTrajectory(
          Autos.runToPlace(s_Swerve.getPose()))
      )));
    */

    /*Line up command */
    driveController.R1().and(driveController.L1()).onTrue(
        Autos.followTrajectory(Autos.lineUp(s_Swerve.getPose()))
          //.andThen(Commands.run(() -> arm.setState(stationSelector.getArmState())))
          //.until(() -> arm.placed())
          //.andThen(new InstantCommand(() -> arm.setState(Position.TRANSIT)))
    );
        


    /* Arm Intake Button Commands */
    specialsController.triangle()
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.circle()
      .onTrue(new InstantCommand(() -> arm.setState(Position.UPCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.cross()
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANCUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.square()
      .onTrue(new InstantCommand(() -> arm.setState(Position.CUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    //Place on predetermined spot kind of
    specialsController.R1()
      .onTrue(new InstantCommand(() -> arm.setState(stationSelector.getArmState())))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    specialsController.L1()
      .onTrue(Commands.run(() -> arm.setState(stationSelector.getArmState()))
      .until(() -> arm.getIntakeEncoder() > arm.intakePlacePos()+3)
      .andThen(Commands.run(() -> arm.setState(Position.TRANSIT))
      .until(() -> Math.abs(arm.getArmAngle()) < 5)));

    testingController.button(3)
      .onTrue(new InstantCommand(() -> arm.setState(Position.AUTOCUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(4)
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANSLIDE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(5)
      .onTrue(new InstantCommand(() -> arm.setState(Position.TIPCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(6)
      .onTrue(new InstantCommand(() -> arm.setState(Position.SHOOTCUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));
/*
    testingController.button(3)
      .onTrue(new InstantCommand(() -> arm.setState(Position.HIGHPLACE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));
    

    testingController.button(4)
      .onTrue(new InstantCommand(() -> arm.setState(Position.MIDPLACE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(9)
      .onTrue(new InstantCommand(() -> arm.setState(Position.UPCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(10)
      .onTrue(new InstantCommand(() -> arm.setState(Position.CUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(12)
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANCUBE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(7)
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANCONE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));

    testingController.button(8)
      .onTrue(new InstantCommand(() -> arm.setState(Position.HUMANSLIDE)))
      .onFalse(new InstantCommand(() -> arm.setState(Position.TRANSIT)));
    */
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
