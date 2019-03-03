/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.drivetrain.DriveForDistance;
import frc.robot.commands.drivetrain.TurnToDegree;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.hablifter.SetPosition;
import frc.robot.commands.multiarm.CargoCollect;
import frc.robot.commands.multiarm.CargoEject;
import frc.robot.commands.multiarm.HatchFlapDown;
import frc.robot.commands.multiarm.HatchFlapUp;
import frc.robot.tekerz.utilities.DpadButton;
import frc.robot.tekerz.utilities.TriggerButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    XboxController dipStick = new XboxController(0);
    XboxController ripStick = new XboxController(1);

    static final int A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4, LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
            BACK = 7, START = 8, RIGHT_THUMBSTICK_BUTTON = 9, LEFT_THUMBSTICK_BUTTON = 10, LEFT_TRIGGER = 2,
            RIGHT_TRIGGER = 3;

    public OI() {
        TriggerButton ejectCargo = new TriggerButton(dipStick, LEFT_TRIGGER) ;
        ejectCargo.whileHeld(new CargoEject());

        TriggerButton collectCargo = new TriggerButton(dipStick, RIGHT_TRIGGER);
        collectCargo.whileHeld(new  CargoCollect());

        Button hPFlapDown = new JoystickButton(dipStick, A_BUTTON);
        hPFlapDown.whenPressed(new HatchFlapDown());
        
        Button hPFlapUp = new JoystickButton(dipStick, B_BUTTON);
        hPFlapUp.whenPressed(new HatchFlapUp());

        Button elevatorToFiveInches = new JoystickButton(dipStick, Y_BUTTON);
        elevatorToFiveInches.whenPressed(new SetHeight(5));

        Button habArmToMinusTen = new JoystickButton(dipStick, X_BUTTON);
        habArmToMinusTen.whenPressed(new SetPosition(-10));

        Button driveFive = new DpadButton(dipStick, 0, 0);
        driveFive.whenPressed(new DriveForDistance(5.0, 0.3));
        
        Button driveFiveEvilTwin = new DpadButton(dipStick, 0, 180);
        driveFiveEvilTwin.whenPressed(new DriveForDistance(-5.0, 0.3));

        Button rotateFortyEight = new DpadButton(dipStick, 0, 90);
        rotateFortyEight.whenPressed(new TurnToDegree(48.0, 0.3));

        Button rotateFortyEightEvilTwin = new DpadButton(dipStick, 0, 270);
        rotateFortyEightEvilTwin.whenPressed(new TurnToDegree(-48.0, 0.3));
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

    }