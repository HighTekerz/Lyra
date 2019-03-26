/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.tekerz.utilities.L;

public class TurnToDegree extends Command {
  private double 
    relativeDegreeToReach,
	degreeToReach,
	speed,
    currentAngle,
    acceptableError = 2;

	/**
	 * 
	 * @param targetDegree Positive for a right turn, Negative for left
	 * @param speed any double between 0.0 and 1.0 (negatives have no effect on the code)
	 */
	public TurnToDegree(double targetDegree, double speed) {
		this.speed = speed;
		this.degreeToReach = targetDegree;
		requires(Robot.Subsystems.drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
    	L.ogCmdInit(this);
		relativeDegreeToReach = degreeToReach + Robot.Subsystems.drivetrain.getAngle();
		SmartDashboard.putNumber("Degree to Reach", relativeDegreeToReach);
		}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {			
		if (relativeDegreeToReach > Robot.Subsystems.drivetrain.getAngle()) {
			speed = -Math.abs(speed);
		}
		else {
			speed = Math.abs(speed);
		}

		currentAngle = Robot.Subsystems.drivetrain.getAngle();

		Robot.Subsystems.drivetrain.arcadeDrive(0, speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
    if (currentAngle > relativeDegreeToReach - acceptableError &&
        currentAngle < relativeDegreeToReach + acceptableError) {
			System.out.println("TurnToDegree Is Finished at: " + currentAngle);
			return true;
		} else {
			System.out.println("TurnToDegree Is NOT Finished at: " + currentAngle);
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		relativeDegreeToReach = degreeToReach;
		Robot.Subsystems.drivetrain.arcadeDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
  protected void interrupted() {
    L.ogCmdInterrupted(this);
  }
}