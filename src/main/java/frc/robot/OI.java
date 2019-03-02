/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.multiarm.CargoCollect;
import frc.robot.commands.multiarm.CargoEject;
import frc.robot.tekerz.utilities.TriggerButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    XboxController dipStick = new XboxController(0);

    static final int A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4, LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
            BACK = 7, START = 8, RIGHT_THUMBSTICK_BUTTON = 9, LEFT_THUMBSTICK_BUTTON = 10, LEFT_TRIGGER = 2,
            RIGHT_TRIGGER = 3;

    public OI() {
        TriggerButton leftTriggerButton = new TriggerButton(dipStick, LEFT_TRIGGER) ;
        leftTriggerButton.whileHeld(new CargoEject());

        TriggerButton rightTriggerButton = new TriggerButton(dipStick, RIGHT_TRIGGER);
        rightTriggerButton.whileHeld(new  CargoCollect());
    }

    public double getLeftStickY() {
        return -dipStick.getRawAxis(1);
    }

    public double getRightStickY() {
        return -dipStick.getY(Hand.kRight);
    }

    public double getRightStickX() {
        return dipStick.getRawAxis(4);
    }

    public boolean getButtonA() {
        return dipStick.getAButton();
    }
    
    public boolean getButtonB() {
        return dipStick.getBButton();
    }

    public boolean getButtonY() {
        return dipStick.getYButton();
    }

    public boolean getButtonX() {
        return dipStick.getXButton();
    }


}