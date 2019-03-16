/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Cmd_AutoDrive;
import frc.robot.commands.Cmd_ResetSensPos;
import frc.robot.commands.Cmd_TrackCube;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick Gamepad = new Joystick(0);
	Joystick Arcadepad = new Joystick(1);
	
	JoystickButton rt = new JoystickButton(Gamepad, 8);
	JoystickButton y = new JoystickButton(Gamepad, 4);
	JoystickButton b = new JoystickButton(Gamepad, 3);
	JoystickButton x = new JoystickButton(Gamepad, 1);
	
	public OI() {
		y.whenPressed(new Cmd_AutoDrive(12, .4, .2));
		b.whenPressed(new Cmd_AutoDrive(-12, .4, .2));
		x.whenPressed(new Cmd_ResetSensPos());

	}
	
	
	
	
	
	
	public boolean getButtonState(int btn) {
		return Gamepad.getRawButton(btn);
	}

	public Joystick getGamepad() {
		return Gamepad;
	}
	

	public Joystick getArcadepad() {
		return Arcadepad;
	}
}
