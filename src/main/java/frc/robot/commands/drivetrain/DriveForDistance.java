/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.tekerz.utilities.L;

public class DriveForDistance extends Command {
  Drivetrain dt = Robot.Subsystems.drivetrain;
  Double 
    inches,
    speed, 
    tickTargetLeft, 
    tickTargetRight;

  boolean reverse;

  public DriveForDistance(Double inches, double speed) {
    requires(dt);
    this.inches = inches;
    this.speed = speed;
    if (inches < 0) {
      reverse = true;
      this.speed = -this.speed;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    L.ogCmdInit(this);
    dt.startBrakeMode();
    tickTargetLeft = (inches * Drivetrain.TICKS_PER_INCH) + dt.getEnc(true);
    tickTargetRight = (inches * Drivetrain.TICKS_PER_INCH) + dt.getEnc(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    dt.arcadeDrive(speed, 0.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (reverse) {
      if (dt.getEnc(true) <= tickTargetLeft
          && dt.getEnc(false) <= tickTargetRight) {
        return true;
      } else {
        log();
        return false;
      }
    } else{
      if (dt.getEnc(true) >= tickTargetLeft
      && dt.getEnc(false) >= tickTargetRight) {
    return true;
  } else {
    log();
    return false;
  }

    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    L.ogCmdEnd(this);
    dt.arcadeDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    L.ogCmdInterrupted(this);
  }

  public void log() {
    if (dt.getEnc(true) < tickTargetLeft) {
      L.og("I have not reached LEFT target. currently: " + dt.getEnc(true));
    }
    if (dt.getEnc(false) < tickTargetRight) {
      L.og("I have not reached RIGHT target. currently: " + dt.getEnc(false));
    }
  }
}