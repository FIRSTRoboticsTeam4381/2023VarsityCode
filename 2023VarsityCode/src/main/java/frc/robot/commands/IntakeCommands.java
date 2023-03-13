package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.math.Conversions;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class IntakeCommands {
    
    private ArmAngleSubsystem arm;
    private ElevatorSubsystem elevator;
    private WristSubsystem wrist;

    public IntakeCommands(ArmAngleSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist){
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
    }
    private double prevArmAngle = 0;

    public FunctionalCommand armToAngle(double angle){
        return new FunctionalCommand(
            () -> Commands.print("START ARM"), //Init
            () -> arm.setArmAngle(Conversions.degreesToArmEncoder(angle)), //Execute
            interrupted -> Commands.print("END ARM"), //OnEnd
            () -> Math.abs(Conversions.armEncoderToDegrees(arm.getArmAngle()) - angle) < 2, //IsFinished
            arm //Requirement
        );
    }

    public FunctionalCommand elevatorToHeight(double height){
        return new FunctionalCommand(
            () -> Commands.print("START ELEVATOR"), //Init
            () -> elevator.setElevator(height), //Execute
            interrupted -> Commands.print("END ELEVATOR"), //OnEnd
            () -> Math.abs(elevator.getElevateHeight() - height) < 2, //IsFinished
            elevator //Requirement
        );
    }

    public FunctionalCommand wristToPosition(double angle){
        return new FunctionalCommand(
            () -> Commands.print("START WRIST"), //Init
            () -> wrist.setWristAngle(Conversions.degreesToWristEncoder(angle)), //Execute
            interrupted -> Commands.print("END WRIST"), //OnEnd
            () -> Math.abs(Conversions.wristEncoderToDegrees(wrist.getWristPos()) - angle) < 5, //IsFinished
            wrist //Requirement
        );
    }

    public FunctionalCommand placeIntake(){
        return new FunctionalCommand(
            () -> Commands.none(), 
            () -> wrist.setIntakeSpeed(1), 
            interrupted -> wrist.setIntakeSpeed(0), 
            () -> wrist.getIntakeVelocity() > 3000, 
            wrist
        );
    }

    /**
     * Runs the robot to place an object
     * @param state double array state [place angle for arm, elevator distance, wrist angle]
     * @return Sequential Command
     */
    public SequentialCommandGroup placeElevator(double[] placeState){
        return new SequentialCommandGroup(
            armToAngle(placeState[0]),
            new ParallelCommandGroup(
                elevatorToHeight(placeState[1]),
                wristToPosition(placeState[2])
            ),
            placeIntake(),
            new ParallelCommandGroup(
                wristToPosition(0),
                elevatorToHeight(0)
            ),
            armToAngle(0)
            
        );
    }


    public SequentialCommandGroup autoPlaceElevator(double[] state){
        return new SequentialCommandGroup(
            armToAngle(state[0]),
            new ParallelCommandGroup(
                elevatorToHeight(state[1]),
                wristToPosition(state[2])
            ),
            placeIntake(),
            wristToPosition(0),
            elevatorToHeight(0),
            armToAngle(0)
            
        );
    }

    public ParallelCommandGroup prePlace(double[] state){
        return new ParallelCommandGroup(
            new InstantCommand(() -> new InstantCommand(() -> wrist.setIntakeSpeed(-0.1))),
            armToAngle(state[0]),
            wristToPosition(state[2])
        );
    }

    /**
     * Runs the robot to intake an item
     * @param state double array state [angle for arm, elevator distance, wrist angle]
     * @return Sequential command
     */
    public ParallelCommandGroup intakePosition(double[] state){
        return new ParallelCommandGroup(
            armToAngle(state[0]),
            elevatorToHeight(state[1]),
            wristToPosition(state[2]),
            new InstantCommand(() -> wrist.setIntakeSpeed(-1)),
            new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) > 300)
        );
    }

    public SequentialCommandGroup uprightConeIntake(double[] state){
        return new SequentialCommandGroup(
            wristToPosition(state[2]),
            new ParallelCommandGroup(
                armToAngle(state[0]),
                elevatorToHeight(state[1]),
                new InstantCommand(() -> wrist.setIntakeSpeed(-1))
            ),
            new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) > 300)
        );
    }

    /**
     * Runs the robot back to transit after intaking
     * @param holdPower intake holding power
     * @return Sequential Command
     */
    public ParallelCommandGroup returnToHome(double holdPower){
        return new ParallelCommandGroup(
            new InstantCommand(() -> wrist.setIntakeSpeed(holdPower)),
            wristToPosition(0),
            elevatorToHeight(0),
            armToAngle(0)
        );
    }

    public ParallelCommandGroup autoReturnToHome(double holdPower){
        return new ParallelCommandGroup(
            new InstantCommand(() -> wrist.setIntakeSpeed(holdPower)),
            wristToPosition(0),
            elevatorToHeight(0),
            armToAngle(50.8)
        );
    }


}