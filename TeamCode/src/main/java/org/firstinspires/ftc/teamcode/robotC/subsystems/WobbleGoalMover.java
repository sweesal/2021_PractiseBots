package org.firstinspires.ftc.teamcode.robotC.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotC.RobotMapBotC;

public class WobbleGoalMover {
    private final DcMotor ringArm = RobotMapBotC.ringArm;
    private static boolean isIntakeTriggered = false;

    private static final double intakePower = 0.5; // Test value

    public WobbleGoalMover() {
        ringArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntake (boolean isBtnPressed, boolean isRevBtnPressed) {
        if (isBtnPressed)
            ringArm.setPower(intakePower);//intake power
        else if (isRevBtnPressed)
            ringArm.setPower(-intakePower);
        else
            ringArm.setPower(0);//stop intake
    }
}
