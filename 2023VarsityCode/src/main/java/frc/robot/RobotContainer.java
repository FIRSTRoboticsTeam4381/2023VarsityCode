// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
 //private final Trigger lime = controller.circle();

  public static StationSelector stationSelector;
  private final Trigger leftDpad = specialsController.povLeft();
  private final Trigger rightDpad = specialsController.povRight();
  private final Trigger topDpad = specialsController.povUp();
  private final Trigger bottomDpad = specialsController.povDown();
  private final Trigger circle = driveController.circle();
  private final Trigger square = driveController.square();
  private final Trigger triangle = driveController.triangle();
  private final Trigger rBumper = driveController.R1();
  private final Trigger lBumper = driveController.L1();
  
  /* Subsystems */
  public static final Swerve s_Swerve = new Swerve();

  //Auto Chooser
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  //Change this and see what happens. Like auto for teleop.
  //private boolean openLoop = true;

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

    SmartDashboard.putData(m_AutoChooser);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

    stationSelector = new StationSelector(DriverStation.getAlliance());

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //Button to reset swerve odometry and angle
    zeroSwerve
      .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0))
      .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));


    
    triangle.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(
      Autos.goToPoint(9.9, 3.5)
    )));
    

    leftDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("L")));
    rightDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("R")));
    topDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("T")));
    bottomDpad.onTrue(new InstantCommand(() -> stationSelector.addStroke("B")));
    circle.onTrue(new InstantCommand(() -> stationSelector.clearKeystroke()));

    rBumper.and(lBumper).onTrue(
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(
        Autos.followTrajectory(Autos.runToPlace(s_Swerve.getPose())
      ))));

      /**
       * Note to self:
       * Teleop Swerve is a default command, meaning anything scheduled that uses drive will take over
       * Hence we can have a driver handoff button that will schedule a swervecontroller command to run a trajectory to drive in to the april
       * tag or specified location based off of the april tag.  We might be doing vision without green lights!
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
