package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase{
    private Swerve s_Swerve;
    private double kP = 0.01, kI = 0.0, kD = 0;
    private PIDController balancePID = new PIDController(kP, kI, kD);

    private Translation2d translation;

    public Balance(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute(){
        double yAxis = MathUtil.clamp(balancePID.calculate(s_Swerve.getPitch(), 0.0), -0.2, 0.2);

        translation = new Translation2d((Math.abs(s_Swerve.getPitch()) > 4)?yAxis:0, 0.0).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, 0, false, true);
    }
}
