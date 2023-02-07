package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase{
    
    private CANSparkMax armTilt1;
    private CANSparkMax armTilt2;
    private CANSparkMax armExtension;

    private CANSparkMax wristAngle;
    private CANSparkMax intake;

    public IntakeArm(){
        armTilt1 = new CANSparkMax(Constants.IntakeArm.armTilt1CAN, MotorType.kBrushless);
        armTilt2 = new CANSparkMax(Constants.IntakeArm.armTilt2CAN, MotorType.kBrushless);
        armExtension = new CANSparkMax(Constants.IntakeArm.armExtensionCAN, MotorType.kBrushless);
        
        wristAngle = new CANSparkMax(Constants.IntakeArm.wristAngleCAN, MotorType.kBrushless);
        intake = new CANSparkMax(Constants.IntakeArm.intakeCAN, MotorType.kBrushless);
    }

    @Override
    public void periodic(){
        armTilt1.set(0);
        armTilt2.set(0);
        armExtension.set(0);
        wristAngle.set(0);
        intake.set(0);
    }
}
