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
            () -> Math.abs(Conversions.armEncoderToDegrees(arm.getArmAbsolute()) - angle) < 2, //IsFinished
            arm //Requirement
        );
    }

    public FunctionalCommand elevatorToHeight(double height){
        return new FunctionalCommand(
            () -> Commands.print("START ELEVATOR"), //Init
            () -> elevator.setElevator(height), //Execute
            interrupted -> Commands.print("END ELEVATOR"), //OnEnd
            () -> Math.abs(elevator.getElevateHeight() - height) < 3, //IsFinished
            elevator //Requirement
        );
    }

    public FunctionalCommand wristToPosition(double angle){
        return new FunctionalCommand(
            () -> Commands.print("START WRIST"), //Init
            () -> wrist.setWristAngle(Conversions.degreesToWristEncoder(angle)), //Execute
            interrupted -> Commands.print("END WRIST"), //OnEnd
            () -> Math.abs(Conversions.wristEncoderToDegrees(wrist.getWristPos()) - angle) < 8, //IsFinished
            wrist //Requirement
        );
    }

    public FunctionalCommand placeIntake(double speed){
        return new FunctionalCommand(
            () -> Commands.none(), 
            () -> wrist.setIntakeSpeed(speed), 
            interrupted -> wrist.setIntakeSpeed(0), 
            () -> wrist.getIntakeVelocity() > speed*3000, 
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
                new WaitCommand(placeState[3]).andThen(wristToPosition(placeState[2])),
                elevatorToHeight(placeState[1])
            ),
            placeIntake(placeState[4]),
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
            placeIntake(state[4]),
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
    public SequentialCommandGroup intakePosition(double[] state){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                armToAngle(state[0]),
                elevatorToHeight(state[1]),
                wristToPosition(state[2]),
                new InstantCommand(() -> wrist.setIntakeSpeed(-1)),
                new WaitUntilCommand(() -> Math.abs(wrist.getIntakeVelocity()) > 300)
            )
            //new InstantCommand(() -> arm.resetArm())
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
    public SequentialCommandGroup returnToHome(double holdPower){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> wrist.setIntakeSpeed(holdPower)),
                wristToPosition(0),
                elevatorToHeight(0),
                armToAngle(0)
            )
            //new InstantCommand(() -> arm.resetArm())
        );
    }

    public ParallelCommandGroup autoReturnToHome(double holdPower){
        return new ParallelCommandGroup(
            new InstantCommand(() -> wrist.setIntakeSpeed(holdPower)),
            wristToPosition(0),
            elevatorToHeight(0),
            armToAngle(0)
        );
    }



    //Better Placing
    public SequentialCommandGroup preplaceElevator(double[] placeState){
        return new SequentialCommandGroup(
            armToAngle(placeState[0]),
            new ParallelCommandGroup(
                new WaitCommand(placeState[3]).andThen(wristToPosition(placeState[2])),
                elevatorToHeight(placeState[1])
            )    
        );
    }

    public SequentialCommandGroup placeAndReturn(double[] placeState){
        return new SequentialCommandGroup(
            placeIntake(placeState[4]),
            new ParallelCommandGroup(
                wristToPosition(0),
                elevatorToHeight(0)
            ),
            armToAngle(0)
        );
    }


}
