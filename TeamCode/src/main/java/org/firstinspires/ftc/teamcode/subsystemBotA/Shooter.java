package org.firstinspires.ftc.teamcode.subsystemBotA;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMapBotA;

public class Shooter {

    private final DcMotor shooter = RobotMapBotA.shooter;
    private final DcMotor elevator = RobotMapBotA.elevator;
    private final Servo trigger = RobotMapBotA.trigger;
    private final Servo slope = RobotMapBotA.slope;
    private final DigitalChannel triggerSwitchIn = RobotMapBotA.triggerSwitchIn;
    private final DigitalChannel triggerSwitchOut = RobotMapBotA.triggerSwitchOut;

    private static boolean isShooting = false; //default state;

    public Shooter () {
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elevator.resetDeviceConfigurationForOpMode();
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trigger.setDirection(Servo.Direction.REVERSE);
        slope.setDirection(Servo.Direction.REVERSE);
        triggerSwitchIn.setMode(DigitalChannel.Mode.INPUT);
        triggerSwitchOut.setMode(DigitalChannel.Mode.INPUT);
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

    private void setTrigger(double inputPos) {
        double pos = Range.clip(inputPos, -0.95, 0.95);
        trigger.setPosition(pos);
    }

    public void setTrigger (boolean isBtnPressed) {
        if (isBtnPressed)
            setTrigger(0.175);
        else
            setTrigger(0.4);
    }

    // This is the method for controlling the slope servo
    // so as to adjust the shooting angle.
    // Current value is 0.45 - 0.52, as the code below.
    public void ctrlSlope (double input) {
        setSlope(Range.clip(input, 0, 1)*0.05 + 0.45);
    }

    // Get the value of the limit switch.
    public boolean getSwitch () {
        return triggerSwitchOut.getState();
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
