package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm;

public class ArmControl extends CommandBase{

    private IntakeArm intakeArm;

    private double armAngle;
    private double armExtension;

    public ArmControl(IntakeArm intakeArm){
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void execute(){
        
    }
    
}
