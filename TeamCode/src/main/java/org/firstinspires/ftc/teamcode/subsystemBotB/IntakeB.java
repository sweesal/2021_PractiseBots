package org.firstinspires.ftc.teamcode.subsystemBotB;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMapBotA;
import org.firstinspires.ftc.teamcode.RobotMapBotB;

public class IntakeB {

    private final DcMotor intake = RobotMapBotB.intake;
    private final DcMotor rotatePlate = RobotMapBotB.rotatePlate;
    private static boolean isIntakeTriggered = false;

    private static final double intakePower = 0.9;

    public IntakeB() {
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        rotatePlate.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntakeFree (boolean isBtnPressed, double setPower) {
        if (isBtnPressed) {
            intake.setPower(Range.clip(setPower, -0.99, 0.99));
        } else {
            intake.setPower(0);
        }
    }

    public void setIntake (boolean isBtnPressed, boolean isRevBtnPressed) {
//        if (isBtnPressed)
//            isIntakeTriggered = !isIntakeTriggered;
//        if (isIntakeTriggered)
        if (isBtnPressed) {
            intake.setPower(intakePower);//intake power
            rotatePlate.setPower(intakePower*0.5);
        }
        else if (isRevBtnPressed)
            intake.setPower(-intakePower);
        else {
            intake.setPower(0);//stop intake
            rotatePlate.setPower(0);
        }
    }

}
