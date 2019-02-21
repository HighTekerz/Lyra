/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tekerz.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class DpadButton extends Button{

	GenericHID m_joystick;
	int m_POV;
	int m_buttonAngle;

	/**
	 * Treats the different angles of the Dpad as different buttons
	 * 
	 * @param joystick
	 * @param pov this can be 0. There have never been problems with  it being 0.
	 * @param buttonAngle which angle of the Dpad you want your button on.
	 */
	public DpadButton(GenericHID joystick, int pov, int buttonAngle) {
		m_joystick = joystick;
		m_POV = pov;
		m_buttonAngle = buttonAngle;
		}

	public boolean get() {
		if (m_joystick.getPOV(m_POV) == m_buttonAngle) {
			return true;
		}
		else {
			return false;
		}
	}
}