package frc.robot;

import frc.robot.subsystems.IntakeArm;

public class SpecialsHandler {

    public Position pose = Position.TRANSIT;
    public Position prevPose = Position.TRANSIT;
    public IntakeAction intakeAct = IntakeAction.HOLD;
    public Type type = Type.CONE;
    private IntakeArm arm;

    public SpecialsHandler(IntakeArm arm){
        this.arm = arm;
    }

    public void intake(){

    }

    public void place(){
        

    }

    public void setType(Type t){
        type = t;
    }

    public double getAnglePos(){
        return 0;
    }

    public double getElevatorHeight(){
        return 0;
    }

    public double getWristPos(){
        return 0;
    }

    public void update(){
        arm.setArmAngle(getAnglePos());
        arm.setElevator(getElevatorHeight());
        arm.setWristAngle(getWristPos());
    }


    public enum Position{
        TRANSIT,
        PREPLACE,
        HIGHPLACE,
        MIDPLACE,
        UPCONE,
        CUBE,
        AUTOCUBE,
        HUMANCUBE,
        HUMANCONE,
        HUMANSLIDE,
        HYBRID,
        TIPCONE,
        SHOOTMIDCUBE,
        SHOOTHIGHCUBE
    }

    public enum IntakeAction{
        HOLD,
        INTAKE,
        PLACE,
        SHOOT
    }

    public enum Type{
        CUBE,
        CONE
    }

    public final double[] getArmState(Position pos){
        switch(pos){
            case TRANSIT://GOOD
                return new double[] {0,0,0};
            case HIGHPLACE://GOOD - Move up a tad because it hits pylon sometimes
                return new double[] {29.12,-32.0,6583};
            case MIDPLACE://GOOD - Move up a tad because it hits pylon sometimes
                return new double[] {29.5,-14.24,4550};
            case UPCONE://GOOD - Check again - Added little adjust
                return new double[] {54,0,0};
            case CUBE://GOOD
                return new double[] {56.90,-1.07,2091};
            case AUTOCUBE://GOOD
                return new double[] {-54.69,0,-3259};
            case HUMANCUBE://GOOD
                return new double[] {9.98,-11.98,8870};
            case HUMANCONE://GOOD
                return new double[] {13.37,-16.35,8000};
            case HUMANSLIDE://GOOD
                return new double[] {-48.16,0,4631};
            case HYBRID://GOOD
                return new double[] {55.88,0,-826};
            case TIPCONE://GOOD
                return new double[] {-62.09,-2.21,329};
            case SHOOTMIDCUBE://GOOD
                return new double[] {0,0,4000};
            case SHOOTHIGHCUBE://GOOD
                return new double[] {32,-15,-1000};
            default:
                return new double[] {0,0,0};
        }

    }

}
