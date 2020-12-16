package org.firstinspires.ftc.teamcode.subsystemBotB;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMapBotA;
import org.firstinspires.ftc.teamcode.RobotMapBotB;

public class ShooterB {

    private final DcMotor shooter = RobotMapBotB.shooter;
    private final DcMotor elevator = RobotMapBotB.elevator;
    private final Servo trigger = RobotMapBotB.trigger;
    private final Servo slope = RobotMapBotB.slope;
    private final DigitalChannel upperBoundIn = RobotMapBotB.upperBoundIn;
    private final DigitalChannel upperBoundOut = RobotMapBotB.upperBoundOut;
    private final DigitalChannel lowerBoundIn = RobotMapBotB.lowerBoundIn;
    private final DigitalChannel lowerBoundOut = RobotMapBotB.lowerBoundOut;


    private static boolean isShooting = false; //default state;

    public ShooterB() {
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elevator.resetDeviceConfigurationForOpMode();
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trigger.setDirection(Servo.Direction.REVERSE);
        slope.setDirection(Servo.Direction.REVERSE);
        upperBoundIn.setMode(DigitalChannel.Mode.INPUT);
        upperBoundOut.setMode(DigitalChannel.Mode.INPUT);
        lowerBoundIn.setMode(DigitalChannel.Mode.INPUT);
        lowerBoundOut.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setShooter (boolean isBtnPressed) {
        if (isBtnPressed)
            isShooting = !isShooting;
        if (isShooting)
            shooter.setPower(0.99);//shooting power
        else
            shooter.setPower(0);//stop shooting
    }

    private void setSlope (double inputPos) {
        double pos = Range.clip(inputPos, -0.95, 0.95);
        slope.setPosition(pos);
    }

    private void setTrigger (double inputPos) {
        double pos = Range.clip(inputPos, -0.95, 0.95);
        trigger.setPosition(pos);
    }

    public void setTrigger (boolean isBtnPressed) {
        if (isBtnPressed)
            setTrigger(0.7);
        else
            setTrigger(0.2);
    }

    // This is the method for controlling the slope servo
    // so as to adjust the shooting angle.
    // Current value is 0.55 - 0.75, as the code below.
    public void ctrlSlope (double input) {
        setSlope(Range.clip(input, 0, 1)*0.2 + 0.55);
    }

    // Get the value of the limit switch.
    public boolean getSwitch1 () {
        return upperBoundIn.getState();
    }

    public boolean getSwitch2 () {
        return upperBoundOut.getState();
    }

    public boolean getSwitch3 () {
        return lowerBoundIn.getState();
    }

    public boolean getSwitch4 () {
        return lowerBoundOut.getState();
    }

    // Elevator mechanism can only move to one direction if top/button limit was triggered.
    public void setElevator (double input, boolean isAtTop, boolean isAtButton) {
        if (isAtTop)
            elevator.setPower(Range.clip(input, -0.2, 0));
        else if (isAtButton)
            elevator.setPower(Range.clip(input, 0, 0.2));
        else
            elevator.setPower(Range.clip(input, -0.5, 0.5));
    }

    // When Y btn of gamepad was pressed elevator goes up, and goes down if A btn pressed.
    // The elevator will single-directional-maneuverable when top/button limit was triggered.
    public void elevatorMove (boolean cmdUp, boolean cmdDown, boolean isAtTop, boolean isAtButton) {
        if (isAtTop)
            elevator.setPower(cmdDown ? -0.25 : 0);
        else if (isAtButton)
            elevator.setPower(cmdUp ? 0.25 : 0);
        else if (!isAtTop | !isAtButton){
            if (cmdUp)
                elevator.setPower(0.3);
            if (cmdDown)
                elevator.setPower(-0.25);
        } else if (!cmdDown && !cmdUp){
            elevator.setPower(0);
        }
    }

    public double getElevator () {
        return elevator.getCurrentPosition();
    }

}
