/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoNothing;
import frc.robot.commands.AutoLift.StageThreeAutoLift;
import frc.robot.commands.AutoLift.StageThreeKeepLevel;
import frc.robot.commands.AutoLift.StageTwoAutoLift;
import frc.robot.commands.AutoLift.PrepareStageThreeAutoLift;
import frc.robot.commands.AutoLift.ReturnToBasePosition;
import frc.robot.commands.CommandGroups.PanelFeederMode;
import frc.robot.commands.CommandGroups.RetractLegsButRunWheels;
import frc.robot.commands.drivetrain.PowerDrive;
import frc.robot.commands.drivetrain.PowerTurn;
import frc.robot.commands.drivetrain.TurnToDegree;
import frc.robot.commands.elevator.DisableElevator;
import frc.robot.commands.elevator.ResetElevatorEncoder;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.hablifter.ClearEncoder;
import frc.robot.commands.hablifter.KeepLevel;
import frc.robot.commands.hablifter.OverrideClimber;
import frc.robot.commands.hablifter.RetractLegs;
import frc.robot.commands.hablifter.SetHabArmPosition;
import frc.robot.commands.multiarm.BothFlapsDown;
import frc.robot.commands.multiarm.BothFlapsUp;
import frc.robot.commands.multiarm.CargoArmDown;
import frc.robot.commands.multiarm.CargoCollect;
import frc.robot.commands.multiarm.CargoEject;
import frc.robot.commands.multiarm.HPPushWhileHeld;
import frc.robot.commands.multiarm.HatchFingerHold;
import frc.robot.commands.multiarm.HatchFingerRelease;
import frc.robot.commands.multiarm.HatchFlapDown;
import frc.robot.commands.multiarm.HatchFlapUp;
import frc.robot.commands.multiarm.LightOn;
import frc.robot.commands.multiarm.StartHatchPanelMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HabLifter;
import frc.robot.tekerz.utilities.DpadButton;
import frc.robot.tekerz.utilities.TriggerButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
@SuppressWarnings("resource")
public class OI {
    XboxController dipStick = new XboxController(0);
    XboxController ripStick = new XboxController(1);
    Joystick climbButton = new Joystick(2);
    XboxController lipStick = new XboxController(3);

    static final int A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4, LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
            BACK = 7, START = 8, RIGHT_THUMBSTICK_BUTTON = 9, LEFT_THUMBSTICK_BUTTON = 10, LEFT_TRIGGER = 2,
            RIGHT_TRIGGER = 3;

    public OI() {
/**
* Dipstick
*/
        // Button bothFlapsDown = new JoystickButton(dipStick, Y_BUTTON);
        // bothFlapsDown.whenPressed(new BothFlapsDown());
        // bothFlapsDown.whenPressed(new HatchFlapDown());

        // Button bothFlapsUp = new JoystickButton(dipStick, A_BUTTON);
        // bothFlapsUp.whenPressed(new BothFlapsUp());

        // Button cargoIntake = new TriggerButton(dipStick, RIGHT_TRIGGER);
        // cargoIntake.whileHeld(new CargoCollect());

        Button cargoOuttake = new TriggerButton(dipStick, LEFT_TRIGGER);
        cargoOuttake.whileHeld(new HPPushWhileHeld());

        Button slowTurnLeft = new DpadButton(dipStick, 0, 90);
        slowTurnLeft.whileHeld(new PowerTurn(.2));
        // slowTurnLeft.whenPressed(new TurnToDegree(-10, .6));
        
        Button slowTurnRight = new DpadButton(dipStick, 0, 270);
        slowTurnRight.whileHeld(new PowerTurn(-.2));
        // slowTurnRight.whenPressed(new TurnToDegree(10, .6));

        Button slowDriveForward = new DpadButton(dipStick, 0, 0);
        slowDriveForward.whileHeld(new PowerDrive(.05));

        Button slowDriveBack = new DpadButton(dipStick, 0, 180);
        slowDriveBack.whileHeld(new PowerDrive(-.05));

        Button startHatchPanelMode = new JoystickButton(dipStick, RIGHT_BUMPER);
        startHatchPanelMode.whenPressed(new PanelFeederMode());

        Button runLED = new JoystickButton(dipStick, A_BUTTON);
        runLED.whileHeld(new LightOn());


/*
 * Ripstick 
 */

        Button elevatorToLevel1 = new JoystickButton(ripStick, A_BUTTON);
        elevatorToLevel1.whenPressed(new SetElevatorHeight(Elevator.HP_LEVEL_1));

        Button elevatorToLevel2 = new JoystickButton(ripStick, B_BUTTON);
        elevatorToLevel2.whenPressed(new SetElevatorHeight(Elevator.HP_LEVEL_2));

        Button elevatorToLevel3 = new JoystickButton(ripStick, Y_BUTTON);
        elevatorToLevel3.whenPressed(new SetElevatorHeight(Elevator.HP_LEVEL_3));

        Button returnToCease = new DpadButton(ripStick, 0, 180);
        returnToCease.whenPressed(new ReturnToBasePosition());

        Button prepForStage3 = new DpadButton(ripStick, 0, 0);
        prepForStage3.whenPressed(new PrepareStageThreeAutoLift());

        Button overrideClimber = new TriggerButton(ripStick, RIGHT_TRIGGER);
        overrideClimber.whileHeld(new OverrideClimber());

        // Button completeStage3 = new JoystickButton(ripStick, START);
        // completeStage3.whenPressed(new CompleteStageThreeAutoLift());

        Button overrideHPFinger = new TriggerButton(ripStick, LEFT_TRIGGER);
        overrideHPFinger.whileHeld(new HatchFingerRelease());

        Button elevatorDisable = new JoystickButton(ripStick, BACK);
        elevatorDisable.toggleWhenPressed(new DisableElevator());

        Button resetElevatorEncoder = new JoystickButton(ripStick, START);
        resetElevatorEncoder.whenPressed(new ResetElevatorEncoder());

/*
* Climb Button
*/

        // final double 

        Button completeClimb = new JoystickButton(climbButton, 9);
        // completeClimb.whenPressed(new DoNothing());
        completeClimb.whenPressed(new StageThreeKeepLevel());

        Button foldFlaps = new JoystickButton(climbButton, 6);
        foldFlaps.whenPressed(new RetractLegs());

        Button sixInAutoClimb = new JoystickButton(climbButton, 8);
        sixInAutoClimb.whenPressed(new StageTwoAutoLift());


/*
* SmartDashboard
*/

        // SmartDashboard.putData("Hab Arm to 18in End Degrees", new SetHabArmPosition(HabLifter.END_DEGREES_FOR_HAB_CLIMB));
        SmartDashboard.putData("StageThreeKeepLevel", new StageThreeKeepLevel());
        SmartDashboard.putData("clear arm encs", new ClearEncoder());
        SmartDashboard.putData("Stage 2", new StageTwoAutoLift());
        // SmartDashboard.putData("keep level", new KeepLevel(0.0, 0.25));
    }


/*
* DIPSTICK
*/
    public double getLeftStickYDip() {
        return -dipStick.getRawAxis(1);
    }

    public double getLeftStickXDip() {
        return dipStick.getRawAxis(0);
    }

    public double getRightStickYDip() {
        return -dipStick.getRawAxis(5);
    }

    public double getRightStickXDip() {
        return dipStick.getRawAxis(4);
    }


/*
* RIPSTICK
*/
    public double getLeftStickYRip() {
        return -ripStick.getRawAxis(1);
    }

    public double getRightStickYRip() {
        return ripStick.getRawAxis(5);
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


/*
* LIPSTICK
*/
    public double getLeftStickYLip() {
        return -lipStick.getRawAxis(1);
    }

    public boolean getButtonALip() {
        return lipStick.getAButton();
    }

    public boolean getButtonBLip() {
        return lipStick.getBButton();
    }

    public boolean getButtonYLip() {
        return lipStick.getYButton();
    }

/*
* Log
*/
    public void log() {
    }
}