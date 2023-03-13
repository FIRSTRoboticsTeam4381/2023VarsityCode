package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristDefault extends CommandBase{
    public WristDefault(WristSubsystem wrist){
        addRequirements(wrist);
    }
    @Override
    public void execute(){}
}
