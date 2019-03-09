/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // public static int PCM_ON_ARM = 30;
  // public static int PCM_ON_MAIN = 31;

  public static class Talons {
    public static TalonSRX 
      habLifterWheelLead = new TalonSRX(7),
      habLifterWheelFollower = new TalonSRX(8),
      liftLead = new TalonSRX(9),
      liftFollower = new TalonSRX(10),
      intake = new TalonSRX(11);

  }

  public static class Sparks {
    public static CANSparkMax 
      rightMotorLead = new CANSparkMax(2, MotorType.kBrushless),
      rightMotorFollower = new CANSparkMax(4, MotorType.kBrushless),
      leftMotorLead = new CANSparkMax(1, MotorType.kBrushless),
      leftMotorFollower = new CANSparkMax(3, MotorType.kBrushless),
      habLifterArmsLead = new CANSparkMax(5, MotorType.kBrushless),
      habLifterArmsFollower = new CANSparkMax(6, MotorType.kBrushless);

  }

  public static class Pneumatics {
    public static int PCM_ON_ARM = 30;
    public static int PCM_ON_MAIN = 31;

    public static Solenoid
      intakeFlap = new Solenoid(PCM_ON_ARM,3),
      hPFlap = new Solenoid(PCM_ON_ARM,2),
      hPPusher = new Solenoid(PCM_ON_ARM,1),
      hPFinger = new Solenoid(PCM_ON_ARM,0),
      habLifterLegs1 = new Solenoid(PCM_ON_MAIN,0),
      habLifterLegs2 = new Solenoid(PCM_ON_MAIN,1);
  }

  public static class Analogs {
    public static AnalogInput
      a0 = new AnalogInput(0),
      a1 = new AnalogInput(1),
      a2 = new AnalogInput(2),
      a3 = new AnalogInput(3),
      a4 = new AnalogInput(4),
      a5 = new AnalogInput(5),
      a6 = new AnalogInput(6),
      a7 = new AnalogInput(7);
  }

  public static class Pigeon {
    public static PigeonIMU
      imu = new PigeonIMU(RobotMap.Talons.liftFollower);
  }

  public static class Switches {
    public static DigitalInput
      hPSensor0 = new DigitalInput(0),
      hPSensor1 = new DigitalInput(1),
      cargoSensor = new DigitalInput(2);
  }

}
