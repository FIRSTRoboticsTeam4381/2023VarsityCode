package frc.robot;

import frc.robot.ArmPositions.Position;
import frc.robot.ArmPositions.Type;


public final class StationSelector {

    private Position position;
    private Type type;
    private Position prevPosition;

    public StationSelector(Position firstPlace, Type preLoad){
        prevPosition = Position.TRANSIT;
        position = firstPlace;
        type = preLoad;
    }
        
    public void setType(Type t){
        type = t;
    }
    public Type getType(){
        return type;
    }
    
    public void setPos(Position pos){
        prevPosition = position;
        position = pos;
    }
    public Position getPos(){
        if(type == Type.CUBE && position == Position.MIDPLACE){
            return Position.SHOOTMIDCUBE;
        }else if(type == Type.CUBE && position == Position.HIGHPLACE){
            return Position.SHOOTHIGHCUBE;
        }
        return position;
    }
    
    public Position getPrevPosition(){
        return prevPosition;
    }

}