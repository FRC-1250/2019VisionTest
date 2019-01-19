package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Sub_RimeRight extends Subsystem {
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
	
	
	

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
    
    public double getCubeX() {
    	NetworkTableEntry tx = table.getEntry("tx");
    	return  tx.getDouble(0); 
    }
    
    public double getCubeY() {
    	NetworkTableEntry ty = table.getEntry("ty");
    	return ty.getDouble(0);
    }
    
    public double getCubeArea() {
    	NetworkTableEntry ta = table.getEntry("ta");
    	return ta.getDouble(0);
    }
}

