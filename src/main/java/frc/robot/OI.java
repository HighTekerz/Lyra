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
import frc.robot.subsystems.HabLifter;
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
        Button habArmToTop = new JoystickButton(dipStick, Y_BUTTON);
        habArmToTop.whenPressed(new SetPosition(HabLifter.TOP_DEAD_CENTER));

        Button habArmToMinusTen = new JoystickButton(dipStick, X_BUTTON);
        habArmToMinusTen.whenPressed(new SetPosition(-10));

        Button habArmTo18Inch = new JoystickButton(dipStick, B_BUTTON);
        habArmTo18Inch.whenPressed(new SetPosition(HabLifter.START_DEGREES_FOR_HAB_CLIMB));

        Button habArmToDownPos = new JoystickButton(dipStick, A_BUTTON);
        habArmToDownPos.whenPressed(new SetPosition(HabLifter.END_DEGREES_FOR_HAB_CLIMB));

        // Button driveFive = new DpadButton(dipStick, 0, 0);
        // driveFive.whenPressed(new DriveForDistance(5.0, 0.3));
        
        // Button driveFiveEvilTwin = new DpadButton(dipStick, 0, 180);
        // driveFiveEvilTwin.whenPressed(new DriveForDistance(-5.0, 0.3));

        // Button rotateFortyEight = new DpadButton(dipStick, 0, 90);
        // rotateFortyEight.whenPressed(new TurnToDegree(48.0, 0.3));

        // Button rotateFortyEightEvilTwin = new DpadButton(dipStick, 0, 270);
        // rotateFortyEightEvilTwin.whenPressed(new TurnToDegree(-48.0, 0.3));
    }

    public double getLeftStickYRip() {
        return -ripStick.getRawAxis(1);
    }

    public double getRightStickXDip() {
        return dipStick.getRawAxis(4);
    }

    public double getLeftStickYDip(){
        return -dipStick.getRawAxis(1);
    }

    public boolean getButtonARip() {
        return ripStick.getAButton();
      }
    
      public boolean getButtonBRip() {
        return ripStick.getBButton();
      }
    
      public boolean getButtonYRip() {
        return ripStick.getYButton();
      }

    }