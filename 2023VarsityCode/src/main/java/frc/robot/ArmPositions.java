package frc.robot;

public final class ArmPositions {


    public static final double[] getArmState(Position pos){
        switch(pos){
            case TRANSIT://UNTESTED
                return new double[] {0,0,0};
            case HIGHPLACE://UNTESTED
            //OLD {49,-32,84.3}
                return new double[] {50,-30,66,0.2, 1};
            case MIDPLACE://UNTESTED
                return new double[] {45,-13,66, 0, 1};
            case PREPLACECUBE:
                return new double[] {55, 0, 0};
            case PREPLACECONE:
                return new double[] {50,0,0};
            case UPCONE://UNTESTED
                return new double[] {106,-0.15,3.9};
            case CUBE://GOOD
                return new double[] {107,0,26.0};
            case AUTOCUBE://GOOD
                return new double[] {-99,-0.4,-44.6};
            case HUMANCUBE://UNTESTED
                return new double[] {21.7,-10,98};
            case HUMANCONE://UNTESTED
                return new double[] {21.7,-12.75,90};
            case HUMANSLIDE://UNTESTED
                return new double[] {-76.0,0,23.5};
            case HYBRID://UNTESTED
                return new double[] {106,0,0, 0, 0.5};
            case TIPCONE://GOOD - Broke encoder though
                return new double[] {-98,-5.2,-68.1};
            case SHOOTMIDCUBE://UNTESTED
                return new double[] {0,0,45, 0, 0.5};
            case SHOOTHIGHCUBE://UNTESTED
                return new double[] {55,-15.4,0, 0, 0.5};
            case PREBUMP:
                return new double[] {-90, 0, 0};
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
        PREPLACECUBE,
        PREBUMP
    }

    public enum Type{
        CONE,
        CUBE
    }
}
