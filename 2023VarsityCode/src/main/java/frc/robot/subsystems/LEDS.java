package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.ArmPositions.Type;

public class LEDS extends SubsystemBase{

    private CANifier leds;
    private double[] yellow = {0.2,1,0};
    private double[] purple = {0,1,1};
    private double[] blue = {0,0,1};
    private double[] set = {};

    public LEDS() {
        leds = new CANifier(49);
        leds.setLEDOutput(blue[0], LEDChannel.LEDChannelA);
        leds.setLEDOutput(blue[1], LEDChannel.LEDChannelB);
        leds.setLEDOutput(blue[2], LEDChannel.LEDChannelC);
    }

    @Override
    public void periodic(){
        set = (RobotContainer.stationSelector.getType() == Type.CONE)?yellow:purple;
        leds.setLEDOutput(set[0], LEDChannel.LEDChannelA);
        leds.setLEDOutput(set[1], LEDChannel.LEDChannelB);
        leds.setLEDOutput(set[2], LEDChannel.LEDChannelC);
    }
}
