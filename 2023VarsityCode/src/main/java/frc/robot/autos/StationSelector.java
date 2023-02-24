package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm.Position;
import frc.robot.subsystems.IntakeArm.Type;

public final class StationSelector {

    
    private final double[] redSidePlacePoints = {0.4,1.0,1.6,2.2,2.8,3.4,4.0,4.6,5.2};
    private final double[] blueSidePlacePoints = {5.2,4.6,4.0,3.4,2.8,2.2,1.6,1.0,0.4};
    private final double redStagePos = 14.31;
    private final double redPlacePos = 14.73;
    private final double bluePlacePos = 1.66;

    private final double[] elevatorPositions = {};
    private final double[] wristAngles = {};
    private final double[] armAngles = {};

    private Alliance alliance;

    private String station = "";
    private String keyStroke = "";
    private String prevStroke = "   ";

    private int keystrokeNum = 0;

    private int gridNum;
    private int column;
    private int row;

    private int index = 0;

    /**
     * 0 : cone
     * 1 : cube
     * 2 : hybrid
     */
    private IntakeArm.Type type;

    public StationSelector(Alliance alliance){
        this.alliance = alliance;
        gridNum = 0;
        column = 0;
        row = 0;
        type = Type.CONE;
    }



    public void addStroke(String stroke){
        if(keystrokeNum < 3){
            keyStroke = keyStroke + stroke;
            keystrokeNum++;
        }
        if(keystrokeNum == 3){
            prevStroke = keyStroke;
            station = "";
            processStation();
            keyStroke = "";
            keystrokeNum = 0;
        }
    }

    public void processStation(){
        switch(prevStroke.charAt(0)){
            case 'L':
                station += "Left Grid - ";
                index = 1;
                break;
            case 'B':
                station += "Middle Grid - ";
                index = 4;
                break;
            case 'R':
                station += "Right Grid - ";
                index = 7;
                break;
        }
        switch(prevStroke.charAt(1)){
            case 'L':
                station += "Left Col - ";
                index--;
                break;
            case 'B':
                station += "Middle Col - ";
                break;
            case 'R':
                station += "Right Col - ";
                index++;
                break;
        }
        switch(prevStroke.charAt(2)){
            case 'T':
                station += "Top Row - ";
                row = 2;
                break;
            case 'L':
                station += "Middle Row - ";
                row = 1;
                break;
            case 'R':
                station += "Middle Row - ";
                row = 1;
                break;
            case 'B':
                station += "Bottom Row - ";
                row = 0;
                break;
        }

        if(prevStroke.charAt(2) == 'B'){
            type = Type.CONE;
        }else if(prevStroke.charAt(1) == 'B'){
            type = Type.CUBE;
        }else{
            type = Type.CONE;
        }
    }
    
    public String getStation(){
        return station;
    }
    public Type getType(){
        return type;
    }
    public String getStroke(){
        return keyStroke;
    }


    public double[] getRedPlacePoint(){
        return new double[] {redPlacePos, redSidePlacePoints[index]};
    }
    public double[] getBluePlacePoint(){
        return new double[] {bluePlacePos, blueSidePlacePoints[index]};
    }

    public double[] getStagePoint(){
        return new double[] {redStagePos, redSidePlacePoints[index]};
    }

    /**
     * @return A position enum
     */
    public Position getArmState(){
        switch(row){
            case 2:
                return Position.HIGHPLACE;
            case 1:
                return Position.MIDPLACE;
            case 0:
                return Position.UPCONE;
            default:
                return Position.TRANSIT;
        }
    }

    public void clearKeystroke(){
        keystrokeNum = 0;
        station = "";
        keyStroke = "";
    }
    
}

