package frc.robot;

public final class ArmPositions {


    public static final double[] getArmState(Position pos){
        switch(pos){
            case TRANSIT://UNTESTED
                return new double[] {0,0,0};
            case HIGHPLACE://UNTESTED
                return new double[] {50,-33.5,84.3};
            case MIDPLACE://UNTESTED
                return new double[] {50,-15.9,84.3};
            case PREPLACECUBE:
                return new double[] {59.4, 0, 0};
            case PREPLACECONE:
                return new double[] {50,0,84.3};
            case UPCONE://UNTESTED
                return new double[] {100,0,60.4};
            case CUBE://GOOD
                return new double[] {108.4,0,26.0};
            case AUTOCUBE://GOOD
                return new double[] {-101.3,-0.4,-44.6};
            case HUMANCUBE://UNTESTED
                return new double[] {21.7,-10,98};
            case HUMANCONE://UNTESTED
                return new double[] {21.7,-12.5,90};
            case HUMANSLIDE://UNTESTED
                return new double[] {-76.0,0,23.5};
            case HYBRID://UNTESTED
                return new double[] {92.4,-0.1,39.3};
            case TIPCONE://GOOD - Broke encoder though
                return new double[] {-100.9,-5.2,-68.1};
            case SHOOTMIDCUBE://UNTESTED
                return new double[] {59.5,0,0};
            case SHOOTHIGHCUBE://UNTESTED
                return new double[] {59.5,-15.4,0};
            default:
                return new double[] {0,0,0};
        }
    }



    public enum Position{
        TRANSIT,
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
        SHOOTHIGHCUBE,
        PREPLACECONE,
        PREPLACECUBE
    }

    public enum Type{
        CONE,
        CUBE
    }
}
