package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class Cmd_ManualDrive extends Command {
	
	private double xCube;
	private double aCube;
	private double Kp = -0.1;
	private double min_command = 0.05;


	public Cmd_ManualDrive() {
		requires(Robot.s_drivetrain);
		requires(Robot.s_limelight);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Checks clutch state by running shiftState Method
	protected void execute() {
		
		if (Robot.m_oi.getButtonState(2)) {
		xCube = Robot.s_limelight.getCubeX();
		aCube = Robot.s_limelight.getCubeArea();

		double heading_error = -xCube;
		double steering_adjust = 0.0;
			
			if(xCube > 1){
				steering_adjust = Kp * heading_error - min_command;
			}
			if(xCube < 1){
				steering_adjust = Kp * heading_error + min_command;
			}
		
		Robot.s_drivetrain.trackCube(steering_adjust, aCube);
		}
		else {
			Robot.s_drivetrain.drive(Robot.m_oi.getGamepad());
		}
		

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		
	}
}
