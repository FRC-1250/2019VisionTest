package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
/**
 *
 */
public class Cmd_AutoTurn extends Command {

	double angle = 0;
	double upperSpeed = 0;
	double lowerSpeed = 0;
	
    public Cmd_AutoTurn(double angle, double upperSpeed, double lowerSpeed) {
    	requires(Robot.s_drivetrain);
    	this.angle = angle;
    	this.upperSpeed = upperSpeed;
    	this.lowerSpeed = lowerSpeed;
    	Robot.s_drivetrain.resetGyro();
    	Robot.s_drivetrain.accumError = 0;
    }
    
    public Cmd_AutoTurn() {
    	angle = 90;
    	
    	requires(Robot.s_drivetrain);
    	Robot.s_drivetrain.resetGyro();
    	Robot.s_drivetrain.accumError = 0;
    }

    protected void initialize() {
    	Robot.s_drivetrain.resetGyro();
    	setTimeout(2);
    	
    }

    protected void execute() {
    	Robot.s_drivetrain.turn(angle, upperSpeed, lowerSpeed);
    }

    protected boolean isFinished() {
    	//Check Gyro and Encorder distance or timeout
    	return Robot.s_drivetrain.isDoneTurning(angle) || isTimedOut();
    	// return Robot.s_drivetrain.isDoneTurning(angle) || isTimedOut();
    }

    protected void end() {
    	Robot.s_drivetrain.resetSensorPos();
    }

    protected void interrupted() {
    }
}
