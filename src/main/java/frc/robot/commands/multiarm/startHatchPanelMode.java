/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.multiarm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.Rioduino.SetMode;
import frc.robot.subsystems.MultiArm;

public class StartHatchPanelMode extends Command {

  private boolean hadHatchPanel;

  public StartHatchPanelMode() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.Subsystems.multiArm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hadHatchPanel = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.Subsystems.multiArm.hasHP() || hadHatchPanel){
      hadHatchPanel = true;
       Robot.Subsystems.multiArm.setFingerUp();
    } else {
       Robot.Subsystems.multiArm.setFingerDown(); 
    }
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
