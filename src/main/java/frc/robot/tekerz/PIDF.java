/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tekerz;

import edu.wpi.first.wpilibj.Timer;

/**
 * PIDFConstantF is an implementation of a PIDF loop for a steady state F
 * to use: call start, then call loop regularly
 * 
 * how to set F:
 * 
 * Elevator that holds itself at setpoint at constant value:
 * set to motor output that holds posistion
 * 
 * System that requires no ouput at setpoint:
 * set to 0
 * 
 */
public class PIDF {
    enum FType {
        CONSTANT,
        SPEED_DEPENDENT,
        GRAVITY_ARM
    }
    double p,i,d,f;
    PIDF.FType fType = FType.CONSTANT;
    double
        iMax = Double.MAX_VALUE,
        error = 0.0, 
        accumulatedError = 0.0,
        previousError = 0.0,
        changeInError = 0.0,
        setpoint = 0.0,
        motorStopValue = 0.0,
        motorMaxValue = 1.0,
        rampRateMax = 0.0,
        output = 0.0,
        lastOutput = 0.0,
        changeInOutputRate = 0.0,
        elapsedTime = 0.0,
        timeNow = 0.0,
        timeLast = 0.0,
        fPart = 0.0;

    Timer timer = new Timer();
    
    public PIDF(double p, double i, double d) {
        this.setPIDF(p, i, d);
    }

    public void setPIDF(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void setFConstant (double f) {
        this.fType = FType.CONSTANT;
        this.f = f;
    }

    public void setFGravityArm(double f, double sensorAtTop, double sensorAtBottom, double motorValueAt90) {
        this.fType = FType.GRAVITY_ARM;
        this.f = f;
    }

    public void setRampRateInMS (double motorStopSpeed, double motorFullSpeed, double timeForFullRunInMS) {
        this.rampRateMax = (motorFullSpeed - motorStopSpeed) / timeForFullRunInMS;
    }

    public void setIMax(double iMax) {
        this.iMax = iMax;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public void start() {
        accumulatedError = 
            changeInOutputRate =
            previousError = 
            0.0;
        timer.reset();
        timer.start();
        timeLast = timer.get();
    }

    /**
     * 
     * @param sensorData what you are measuring
     * @return motor output
     */
    public double loop(double sensorData) {
        timeNow = timer.get();
        elapsedTime = timeNow - timeLast;
        timeLast = timeNow; // save for next time

        error = setpoint - sensorData;

        accumulatedError += error * elapsedTime;
        if (accumulatedError > iMax) accumulatedError = iMax;

        changeInError = (error - previousError) / elapsedTime;

        switch (this.fType) {
            case SPEED_DEPENDENT:
                //TODO: make this scale correctly
                break;
            case GRAVITY_ARM:
                break;
            case CONSTANT:
                fPart = f;
            default:
                break;
        }

        output = 
            p * error +
            i * accumulatedError +
            d * changeInError +
            fPart;

        changeInOutputRate = (output - lastOutput) / elapsedTime;

        // we need to change output to only increate the amount it is allowed in this length of time
        // and in the direction of change
        if (Math.abs(changeInOutputRate) > rampRateMax) {
            lastOutput +=
                (rampRateMax * elapsedTime) * // amount of change in this timeframe
                (changeInOutputRate / changeInOutputRate); // in the correct direction of change
        }

        return output;
    }
}
