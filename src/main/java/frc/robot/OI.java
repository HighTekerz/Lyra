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
import frc.robot.commands.AutoLift.CompleteStageThreeAutoLift;
import frc.robot.commands.AutoLift.PrepareStageThreeAutoLift;
import frc.robot.commands.AutoLift.ReturnToBasePosition;
import frc.robot.commands.Rioduino.SetMode;
import frc.robot.commands.drivetrain.DriveForDistance;
import frc.robot.commands.drivetrain.TurnToDegree;
import frc.robot.commands.elevator.SetHeight;
import frc.robot.commands.hablifter.SetHabArmPosition;
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
        /**
         * dipstick:
         *  left trigger: outtake
         *  right trigger: intake
         *  A button: Double flap down
         *  B button: Double flap up
         * 
         */

        /**
         * ripStick
         *  A = Level 1
         *  B = Level 2
         *  Y = Level 3
         *  ???: complete climb procedure
         */

        Button returnToCease = new DpadButton(ripStick, 0, 0);
        returnToCease.whenPressed(new ReturnToBasePosition());

        Button prepForStage3 = new DpadButton(ripStick, 0, 180);
        prepForStage3.whenPressed(new PrepareStageThreeAutoLift());
        
        Button rainbowMode = new DpadButton(ripStick, 0, 90);
        rainbowMode.whenPressed(new SetMode("3"));
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