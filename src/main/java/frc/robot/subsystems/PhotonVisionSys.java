package frc.robot.subsystems;
import edu.wpi.first.networktables.*;
public class PhotonVisionSys {
    public NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("limelight");
    public PhotonVisionSys(){
        LLTable.getEntry("camMode").setNumber(0); //sets camera to vision processing mode
        LLTable.getEntry("pipeline").setNumber(0);//sets the pipeline to 0 which is default
    }
}
