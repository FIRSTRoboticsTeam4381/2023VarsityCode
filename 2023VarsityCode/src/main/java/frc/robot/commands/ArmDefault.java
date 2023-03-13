package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAngleSubsystem;

public class ArmDefault extends CommandBase{
    public ArmDefault(ArmAngleSubsystem arm){
        addRequirements(arm);
    }
    @Override
    public void execute(){}
}
