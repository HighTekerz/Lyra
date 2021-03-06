/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoNothing;
import frc.robot.commands.CommandGroups.PlaceAndReverse;
import frc.robot.commands.CommandGroups.Turn180;
import frc.robot.commands.drivetrain.DriveByPorcupine;
import frc.robot.commands.drivetrain.DriveForDistance;
import frc.robot.commands.drivetrain.DriveWithJaci;
import frc.robot.commands.drivetrain.TurnToDegree;
import frc.robot.commands.test.AddTestCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HabLifter;
import frc.robot.subsystems.HabLifterWheels;
import frc.robot.subsystems.MultiArm;
import frc.robot.subsystems.Sensors;
import frc.robot.tekerz.Rioduino;
import frc.robot.tekerz.utilities.L;
import jaci.pathfinder.Waypoint;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static class Subsystems {
    public static Drivetrain drivetrain = new Drivetrain();
    public static Elevator elevator = new Elevator();
    // public static Sensors sensors = new Sensors();
    public static MultiArm multiArm = new MultiArm();
    public static HabLifter habLifter = new HabLifter();
    public static HabLifterWheels habLifterWheels = new HabLifterWheels();
    public static Rioduino rioduino = new Rioduino();
  }

  public static OI oi = new OI();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Do Nothing", new DoNothing());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putData(new AddTestCommands());
    SmartDashboard.putData(Scheduler.getInstance());

    Waypoint[] testPoints  =  new  Waypoint[] {
      new  Waypoint(0.0, 0.0, 0.0),
      new  Waypoint(4.0, 0.0, 0.0),
    };
    // SmartDashboard.putData("test path", new  DriveWithJaci(testPoints));
    // SmartDashboard.putData("Tekerz Path(z)", new DriveByPorcupine(88, 88, 0.5, 0.0, true));
    // SmartDashboard.putData("Cargo Front Auto", new DriveByPorcupine(62, 62, 0.3, 0.0, true));
    SmartDashboard.putData(new DriveForDistance(24.0, 0.4));
    SmartDashboard.putData(new PlaceAndReverse());
    SmartDashboard.putData(new Turn180());
    SmartDashboard.putData(new TurnToDegree(90, 1));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    this.log();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    System.out.println("Gracious Professionalism > Meaningful Life");

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void log() {
    // Robot.Subsystems.sensors.log();
    Robot.Subsystems.elevator.log();
    Robot.Subsystems.multiArm.log();
    Robot.Subsystems.drivetrain.log();
    Robot.Subsystems.habLifter.log();
    Robot.oi.log();
  }

}
