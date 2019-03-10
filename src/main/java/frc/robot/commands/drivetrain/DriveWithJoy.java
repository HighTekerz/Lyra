/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoy extends Command {
  Drivetrain drivetrain = Robot.Subsystems.drivetrain;
  OI oi = Robot.oi;

  private double
    throttle,
    turn;

  public DriveWithJoy() {
    requires(drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    throttle = oi.getRightStickYDip();
    if(Math.abs(throttle) < 0.1){
      throttle = 0;
    }
    
    turn = oi.getLeftStickXDip();
    if(Math.abs(turn) < 0.1){
      turn = 0;
    }

    if(Math.abs(throttle) > 0.9){
      if(throttle < 0){
        throttle = -0.9;
      }
      else{
        throttle = 0.9;
      }
    }

    if(Math.abs(turn) > 0.9){
      if(turn < 0){
        turn = -0.9;
      }
      else{
        turn = 0.9;
      }
    }

    drivetrain.arcadeDrive(Math.pow(throttle, 3), Math.pow(turn, 3));
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
