package org.firstinspires.ftc.teamcode.robotC.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotC.RobotMapBotC;

public class WobbleGoalMover {
    private final DcMotor ringArm = RobotMapBotC.ringArm;
    private final Servo claw = RobotMapBotC.claw;
    private static boolean isIntakeTriggered = false;

    private final ElapsedTime clawTimer;
    private static double clawPosition = 0.2; //default value

    private static final double intakePower = 0.5; // Test value

    public WobbleGoalMover() {
        clawTimer = new ElapsedTime();
        ringArm.setDirection(DcMotorSimple.Direction.FORWARD);
        ringArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void setIntake (boolean isBtnPressed, boolean isRevBtnPressed) {
        if (isBtnPressed)
            ringArm.setPower(intakePower);//intake power
        else if (isRevBtnPressed)
            ringArm.setPower(-intakePower);
        else
            ringArm.setPower(0);//stop intake
    }

    public void setClaw (boolean isBtnPressed) {

    }

    public void setClaw (boolean cmdUp, boolean cmdDown) {
        if(cmdUp && clawTimer.seconds() > 0.1) {
            clawPosition +=0.05;
            clawTimer.reset();
        } else if(cmdDown && clawTimer.seconds() >0.1) {
            clawPosition -=0.05;
            clawTimer.reset();
        }
        claw.setPosition(Range.clip(clawPosition, 0.01, 0.99)); // Default from 0 - 1.
    }
}
