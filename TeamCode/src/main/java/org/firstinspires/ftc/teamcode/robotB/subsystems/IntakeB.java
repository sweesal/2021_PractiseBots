package org.firstinspires.ftc.teamcode.robotB.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.robotB.RobotMapBotB;

public class IntakeB {

    private final DcMotor intake = RobotMapBotB.intake;
    private final DcMotor elevator = RobotMapBotB.elevator;
    private final DcMotor rotatePlate = RobotMapBotB.rotatePlate;
    private final DigitalChannel lowerBoundOut = RobotMapBotB.lowerBoundOut;
    private static boolean isIntakeTriggered = false;

    private static final double intakePower = 0.9;

    public IntakeB() {
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatePlate.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntake (boolean btnIntake, boolean btnReverse) {
        if (btnIntake) {
            if(lowerBoundOut.getState()) elevator.setPower(-0.5);
            else {
                elevator.setPower(0.0);
                intake.setPower(1.0);
                rotatePlate.setPower(1.0);
            }
        } else {
            if (btnReverse) intake.setPower(-1.0);
            else intake.setPower(0.0);
            rotatePlate.setPower(0.0);
        }
    }
}
