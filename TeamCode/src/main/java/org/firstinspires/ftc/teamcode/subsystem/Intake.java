package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotMap;

public class Intake {

    private final DcMotor intake = RobotMap.intake;
    private static boolean isIntakeTriggered = false;

    private static final double intakePower = 0.6;

    public Intake () {
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntake (boolean isBtnPressed) {
        if (isBtnPressed)
            isIntakeTriggered = !isIntakeTriggered;
        if (isIntakeTriggered)
            intake.setPower(0.9);//intake power
        else
            intake.setPower(0);//stop intake
    }

}
