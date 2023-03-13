package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDefault extends CommandBase{
    public ElevatorDefault(ElevatorSubsystem elevator){
        addRequirements(elevator);
    }
    @Override
    public void execute(){}
}