package frc.robot.autos;

import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm.Position;
import frc.robot.subsystems.IntakeArm.Type;

public final class StationSelector {


    private IntakeArm.Position position;
    private IntakeArm.Type type;
    
    public StationSelector(Position firstPlace, Type preLoad){
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
    
}

