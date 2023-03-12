package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.IntakeArm;

public class IntakeCommands {
    
    private IntakeArm arm;

    public IntakeCommands(IntakeArm arm){
        this.arm = arm;
    }

    public FunctionalCommand armToAngle(double angle){
        return new FunctionalCommand(
            () -> Commands.none(), //Init
            () -> arm.setArmAngle(angle), //Execute
            interrupted -> Commands.none(), //OnEnd
            () -> Math.abs(arm.getArmAngle() - angle) < 2, //IsFinished
            arm //Requirement
        );
    }

    public FunctionalCommand elevatorToHeight(double height){
        return new FunctionalCommand(
            () -> Commands.none(), //Init
            () -> arm.setElevator(height), //Execute
            interrupted -> Commands.none(), //OnEnd
            () -> Math.abs(arm.getElevateHeight() - height) < 2, //IsFinished
            arm //Requirement
        );
    }

    public FunctionalCommand wristToPosition(double angle){
        return new FunctionalCommand(
            () -> Commands.none(), //Init
            () -> arm.setWristAngle(angle), //Execute
            interrupted -> Commands.none(), //OnEnd
            () -> Math.abs(arm.getWristPos() - angle) < 2, //IsFinished
            arm //Requirement
        );
    }

    public FunctionalCommand placeIntake(){
        return new FunctionalCommand(
            () -> Commands.none(), 
            () -> arm.setIntakeSpeed(1), 
            interrupted -> arm.setIntakeSpeed(0), 
            () -> arm.getIntakeVelocity() > 3000, 
            arm
        );
    }

    public ParallelCommandGroup place(double distance, double wristAngle){
        return new ParallelCommandGroup(
            elevatorToHeight(distance),
            new WaitUntilCommand(() -> Math.abs(arm.getElevateHeight() - distance) < 8)
                .andThen(wristToPosition(wristAngle))
                .andThen(placeIntake())
        );
    }


}
