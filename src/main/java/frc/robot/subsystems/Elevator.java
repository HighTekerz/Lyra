/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.tekerz.utilities.L;

public class Elevator extends Subsystem {

  public static double MOTOR_HOLD_VALUE = -0.167,

  // measured ticks
  TICKS_PER_INCH = -934.0 / 28.0, //-33.357
  // system length is 30 inches
  MAX_ERROR = 30 * TICKS_PER_INCH,

  STARTING_OFFSET = 18.75,
  Level_1HALF = 5.0,
  HP_FEEDER_PICKUP = 19.5,
  HP_LEVEL_1 = 21.5,
  DRIVE_SLOW_HEIGHT = 24,
  HP_LEVEL_2 = HP_LEVEL_1 + 28.0,
  HP_LEVEL_3 = HP_LEVEL_2 + 28.0,
  CARGO_LEVEL_1 = 21.5,
  CARGO_LEVEL_1dot5 = 38.5,
  CARGO_LEVEL_2 = CARGO_LEVEL_1 + 28.0,
  CARGO_LEVEL_3 = CARGO_LEVEL_2 + 28.0;

  TalonSRX 
    liftLead = RobotMap.Talons.liftLead,
    liftFollower = RobotMap.Talons.liftFollower;

  double 
    p = 0.002,
    i = 0.0,
    d = 0.0,
    loopLengthInSeconds = .02;

  public double
    elevatorTicks;

  private final PIDOutput output = this::setElevatorPIDOutput;
  private final PIDSource input = new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public double pidGet() {
      return getElevatorPosition();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };

  private final PIDController pIDLoop = new PIDController(p, i, d, input, output, loopLengthInSeconds) {
    @Override
    protected double calculateFeedForward() {
      return MOTOR_HOLD_VALUE;
    }
  };

  public Elevator() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    liftLead.configAllSettings(config);
    liftFollower.configAllSettings(config);
    liftFollower.follow(liftLead);
  
    liftLead.setNeutralMode(NeutralMode.Brake);
    liftFollower.setNeutralMode(NeutralMode.Brake);

    // pIDLoop.setOutputRange(-0.6, -0.03715);
    pIDLoop.setOutputRange(-0.6, 0.1);

    SmartDashboard.putData(this);
  }

  @Override
  protected void initDefaultCommand() {
  }

  private void setElevatorPIDOutput(double out) {
    liftLead.set(ControlMode.PercentOutput, out);
    L.ogSD("PID Elevator ouput", out);
  }

  public double getElevatorPosition() {
    return liftLead.getSelectedSensorPosition();
  }

  public void setSetpoint(double setpointInInches) {
    this.pIDLoop.setSetpoint((setpointInInches - STARTING_OFFSET) * TICKS_PER_INCH);
  }

  public void enablePid() {
    this.pIDLoop.enable();
  }

  public void disablePid() {
    this.pIDLoop.disable();
  }

  public void resetEncoder(){
    liftLead.setSelectedSensorPosition(0);
  }

  public void log() {
    elevatorTicks = getElevatorPosition();
    L.ogSD("Elevator Ticks", elevatorTicks);
  }
}
