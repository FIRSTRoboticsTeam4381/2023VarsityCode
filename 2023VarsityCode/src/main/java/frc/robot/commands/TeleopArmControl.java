package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.IntakeArm;

public class TeleopArmControl extends CommandBase{

    private IntakeArm intakeArm;

    private CommandPS4Controller controller;

    public TeleopArmControl(IntakeArm intakeArm, CommandPS4Controller controller){
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);

        this.controller = controller;
    }


    public void transit(){intakeArm.setPosition(IntakeArm.Position.TRANSIT);}
    public void placeHigh(){intakeArm.setPosition(IntakeArm.Position.HIGHPLACE);}
    public void placeMid(){intakeArm.setPosition(IntakeArm.Position.MIDPLACE);}
    public void groundIn(){intakeArm.setPosition(IntakeArm.Position.UPCONE);}
    public void coneGround(){intakeArm.setPosition(IntakeArm.Position.TIPCONE);}
    public void slideStation(){intakeArm.setPosition(IntakeArm.Position.HUMANSLIDE);}
    public void uprightStation(){intakeArm.setPosition(IntakeArm.Position.HUMANUPRIGHT);}


    @Override
    public void execute(){
        controller.triangle().onTrue(new InstantCommand(() -> groundIn())).onFalse(new InstantCommand(() -> transit()));
        controller.cross().onTrue(new InstantCommand(() -> placeHigh())).onFalse(new InstantCommand(() -> transit()));
    }

}
