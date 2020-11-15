package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMap;

public class Shooter {

    private final DcMotor shooter = RobotMap.shooter;
    private final DcMotorEx elevator = RobotMap.elevator;
    private final Servo trigger = RobotMap.trigger;
    private final Servo slope = RobotMap.slope;
    private final DigitalChannel triggerSwitchIn = RobotMap.triggerSwitchIn;
    private final DigitalChannel triggerSwitchOut = RobotMap.triggerSwitchOut;

    private static boolean isShooting = false; //default state;

    public Shooter () {
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.resetDeviceConfigurationForOpMode();
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

    public void setSlope (double inputPos) {
        double pos = Range.clip(inputPos, -0.95, 0.95);
        slope.setPosition(pos);
    }

    public void setTrigger(double inputPos) {
        double pos = Range.clip(inputPos, -0.95, 0.95);
        trigger.setPosition(pos);
    }

    public void setTrigger (boolean isBtnPressed) {
        if (isBtnPressed)
            setTrigger(0.175);
        else
            setTrigger(0.6);
    }


    public void ctrlSlope (double input) {
        setSlope(Range.clip(input, 0, 1)*0.07 + 0.45);
    }

    public boolean getSwitch () {
        return triggerSwitchOut.getState();
    }


    public void setElevator (double input, boolean isAtTop, boolean isAtButton) {
        if (isAtTop)
            elevator.setPower(Range.clip(input, -0.15, 0));
        else if (isAtButton)
            elevator.setPower(Range.clip(input, 0, 0.15));
        else
            elevator.setPower(Range.clip(input, -0.5, 0.5));
    }

    public double getElevator () {
        return elevator.getCurrentPosition();
    }



}
