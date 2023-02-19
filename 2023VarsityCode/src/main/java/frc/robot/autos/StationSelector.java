package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public final class StationSelector {

    
    private final double[] redSidePlacePoints = {0.4,1.0,1.6,2.2,2.8,3.4,4.0,4.6,5.2};
    private final double stagePos = 14.31;
    private final double placePos = 14.73;

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
    private String type;

    public StationSelector(Alliance alliance){
        this.alliance = alliance;
        gridNum = 0;
        column = 0;
        row = 0;
        type = "Cone";
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
            type = "Hybrid";
        }else if(prevStroke.charAt(1) == 'B'){
            type = "Cube";
        }else{
            type = "Cone";
        }
    }
    
    public String getStation(){
        return station;
    }
    public String getType(){
        return type;
    }
    public String getStroke(){
        return keyStroke;
    }


    public double[] getPlacePoint(){
        return new double[] {placePos, redSidePlacePoints[index]};
    }

    public double[] getStagePoint(){
        return new double[] {stagePos, redSidePlacePoints[index]};
    }

    /**
     * @return A double array of [arm angle, elevator position, wrist angle]
     */
    public double[] getArmState(){
        return new double[] {armAngles[index], elevatorPositions[index], wristAngles[index]};
    }

    public void clearKeystroke(){
        keystrokeNum = 0;
        station = "";
        keyStroke = "";
    }
    
}

