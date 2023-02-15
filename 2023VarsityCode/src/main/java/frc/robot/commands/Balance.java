package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase{
    private Swerve s_Swerve;
    private PIDController balancePID;

    private Translation2d translation;

    public Balance(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        balancePID = new PIDController(0.015, 0.0, 0.007);
    }

    @Override
    public void execute(){
        double yAxis = -MathUtil.clamp(balancePID.calculate(s_Swerve.getPitch(), 0.0), -0.12, 0.12);

        translation = new Translation2d(yAxis, 0.0).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, 0, false, true);
    }
}
