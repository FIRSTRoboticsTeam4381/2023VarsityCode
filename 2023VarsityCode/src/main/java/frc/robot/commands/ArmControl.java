package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm;

public class ArmControl extends CommandBase{

    private IntakeArm intakeArm;

    private CommandPS4Controller controller;
    private double armAngle;
    private double armExtension;

    public ArmControl(IntakeArm intakeArm, CommandPS4Controller controller){
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);

        this.controller = controller;
        
    }


    public void transit(){}
    public void placeHigh(){}
    public void placeMid(){}
    public void groundIn(){}
    public void coneGround(){}


    @Override
    public void execute(){
        controller.triangle().onTrue(new InstantCommand(() -> groundIn())).onFalse(new InstantCommand(() -> transit()));
    }
    
}
