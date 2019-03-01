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
public class TriggerButton extends Button {
	
    GenericHID m_joystick;
    int m_axis;

   public TriggerButton(GenericHID joystick, int axis) {
      m_joystick = joystick;
      m_axis = axis;
   }
   public boolean get() {
      if (m_joystick.getRawAxis(m_axis) >= 0.75) {
           return true;
       } else {
           return false;
       }
   }
}