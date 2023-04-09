package frc.robot;

public final class ArmPositions {


    public static final double[] getArmState(Position pos){
        //Shifted stuff about +8degrees when changed head
        switch(pos){
            case TRANSIT://UNTESTED
                return new double[] {0,0,77};
            case HIGHPLACE://UNTESTED
            //OLD {49,-32,84.3}
                return new double[] {50,-30,74,0.2, 1};
            case MIDPLACE://UNTESTED
                return new double[] {45,-13,80, 0, 1};
            case PREPLACECUBE:
                return new double[] {55, 0, 0};
            case PREPLACECONE:
                return new double[] {50,0,0};
            case UPCONE://UNTESTED
            // Old {106,-0.15,3.9}
                return new double[] {98.37,0,30.81, 0};
            case CUBE://GOOD
                return new double[] {107,0,30, 0};
            case AUTOCUBE://GOOD
                return new double[] {-99,-0.4,-44.6, 0};
            case HUMANCUBE://UNTESTED
                return new double[] {21.7,-10,106, 0};
            case HUMANCONE://UNTESTED
            //Old elevator -12.93 Old wrist 83.23
                // UPRIGHT return new double[] {25.27,-14.35,98, 0};
                return new double[] {21.7,-10,106, 0};
            case UPRIGHTOVERRIDE:
                return new double[] {25.27,-14.35,98, 0};
            case HUMANSLIDE://UNTESTED
                return new double[] {-76.0,0,23.5, 0};
            case HYBRID://UNTESTED
                return new double[] {74,0,65, 0, 0.5};
            case TIPCONE://GOOD - Broke encoder though
            //Old{-98,-5.2,-68.1
                return new double[] {113.44,-0.86,13.60, 0};
            case SHOOTMIDCUBE://UNTESTED
                return new double[] {0,0,68, 0, 0.5};
            case SHOOTHIGHCUBE://UNTESTED
                return new double[] {55,-15.4,8, 0, 0.5};
            case PREBUMP:
                return new double[] {-90, 0, 45};
            case SPIT:
                return new double[] {0,0,77, 0, 1};
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
        PREBUMP,
        SPIT,
        UPRIGHTOVERRIDE
    }

    public enum Type{
        CONE,
        CUBE
    }
}
