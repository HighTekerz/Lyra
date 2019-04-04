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
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveWithJaci extends Command {

	private Drivetrain dt = Robot.Subsystems.drivetrain;
	private Waypoint[] points;
	public Trajectory.Config config;
	public Trajectory trajectory;
	public TankModifier modifier;
	public EncoderFollower left;
  public EncoderFollower right;
  double startLeftEnc, startRightEnc;

  
  private final double 
    DELTA_TIME = 0.02, 
    MAX_VEL = 3.0,
    MAX_ACCEL = 2.0,
    MAX_JERK = 0.9,
    WHEELBASE_WIDTH = 25.0 * 0.0254,
    WHEEL_DIAMETER_METERS = 7.75 * 0.0254, //in * in/meter
    P = 0.00005,
    TRANSMISSION_TICKS = 10710;
  private final int TICKS_PER_MOTOR_REV = 1000;
  

  /** used to run a path of points
   *  all the init is in the constructor
   */
  public DriveWithJaci(Waypoint[] points) {
    requires(dt);
    this.points = points;

    config = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_CUBIC, 
      Trajectory.Config.SAMPLES_FAST, 
      DELTA_TIME, 
      MAX_VEL,
      MAX_ACCEL,
      MAX_JERK);

    setupPath();
  }

  private void setupPath() {
    trajectory = Pathfinder.generate(points, config);
    for (Segment s : trajectory.segments) {
      System.out.println(String.format("%.3f %.3f %.3f %.3f", s.x, s.velocity, s.acceleration, s.jerk));
    }

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(WHEELBASE_WIDTH);
		left = new EncoderFollower(modifier.getLeftTrajectory());
    right = new EncoderFollower(modifier.getRightTrajectory());
  }

  private int getLeft() {
    return (int)((dt.getEnc(true)-startLeftEnc) * TRANSMISSION_TICKS);
  }

  private int getRight() {
    return (int)((dt.getEnc(false)-startLeftEnc) * TRANSMISSION_TICKS);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt.resetYaw();
    startLeftEnc = dt.getEnc(true);
    startRightEnc = dt.getEnc(false);
    
    // dt.clearEncoders();
    left.configureEncoder(getLeft(), TICKS_PER_MOTOR_REV, WHEEL_DIAMETER_METERS);
		right.configureEncoder(getRight(), TICKS_PER_MOTOR_REV, WHEEL_DIAMETER_METERS);
		left.configurePIDVA(P, 0.0, 0.0, (1 / MAX_VEL), 0.0);
    right.configurePIDVA(P, 0.0, 0.0, (1 / MAX_VEL), 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double l = left.calculate(getLeft());
    double r = left.calculate(getRight());
    
    double gyro_heading = dt.getAngle();
    double desired_heading = Pathfinder.r2d(left.getHeading());
    double angleDifference = Pathfinder.boundHalfDegrees(gyro_heading - desired_heading);
    double turn = 0.8 * angleDifference / 90.0;

    SmartDashboard.putNumber("jaci L", l);
    SmartDashboard.putNumber("jaci R", r);
    SmartDashboard.putNumber("jaci T", turn);

    System.out.println(String.format("%3f %3f %3f", l, getLeft() / this.TRANSMISSION_TICKS, dt.getEncLSpeed()));

    dt.setWheelSpeed(l, r);
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
