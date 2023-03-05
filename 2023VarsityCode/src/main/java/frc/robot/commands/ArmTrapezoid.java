package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.IntakeArm;

public class ArmTrapezoid extends TrapezoidProfileCommand{

    public ArmTrapezoid(double angle, IntakeArm intakeArm){
        super(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    0.1,
                    0.1
                ),
                //Goal State
                new TrapezoidProfile.State(
                    angle,
                    0
                ),
                //Initial State
                new TrapezoidProfile.State(
                    intakeArm.getArmAngle(),
                    intakeArm.getArmVelocity()
                )),
            setPointState -> intakeArm.setArmAngle(setPointState.position),

            intakeArm
        );

    }
    
}
