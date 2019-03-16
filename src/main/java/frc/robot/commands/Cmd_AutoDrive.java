package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *
 */
public class Cmd_AutoDrive extends Command {

	int distance = 0;
	double upperSpeed;
    double lowerSpeed;
    float sign;

    public Cmd_AutoDrive(int distance, double upperSpeed, double lowerSpeed) {
    	requires(Robot.s_drivetrain);
    	this.distance = distance;
    	this.upperSpeed = upperSpeed;
    	this.lowerSpeed = upperSpeed;
    }
    

    protected void initialize() {
    	Robot.s_drivetrain.resetSensorPos();
    	Robot.s_drivetrain.resetGyro();
    	Robot.s_drivetrain.setSetpointPos(distance);
    	setTimeout(5);
    }


    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.s_drivetrain.driveToPos(upperSpeed, lowerSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        sign = Math.signum(distance);
        
        if (sign == 1){
            return Robot.s_drivetrain.isDoneDriving() || isTimedOut();
        }
        if (sign == -1){
            return Robot.s_drivetrain.isDoneDrivingBack() || isTimedOut();
        }
        else{
            return false;
        }
        }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
