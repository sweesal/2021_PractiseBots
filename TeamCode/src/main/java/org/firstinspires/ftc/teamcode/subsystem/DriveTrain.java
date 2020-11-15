package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.Arrays;

public class DriveTrain {

    private final DcMotor leftFront = RobotMap.leftFront;
    private final DcMotor leftRear = RobotMap.leftRear;
    private final DcMotor rightFront = RobotMap.rightFront;
    private final DcMotor rightRear = RobotMap.rightRear;

    public DriveTrain () {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveMecanum(double xSpeed, double zRotation, double yTranslation) {
        xSpeed = limit(xSpeed);
        //xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = limit(zRotation);
        //zRotation = applyDeadband(zRotation, m_deadband);

        double[] leftMotorOutput = new double[2];
        double[] rightMotorOutput = new double[2];

        if (xSpeed != 0.0){
            leftMotorOutput [0] = -xSpeed;
            leftMotorOutput [1] = -xSpeed;
            rightMotorOutput [0] = xSpeed;
            rightMotorOutput [1] = xSpeed;
        }

        else if (zRotation != 0.0){
            leftMotorOutput [0] = -zRotation;
            leftMotorOutput [1] = -zRotation;
            rightMotorOutput [0] = -zRotation;
            rightMotorOutput [1] = -zRotation;
        }

        else if (yTranslation != 0.0){
            leftMotorOutput[0] = yTranslation;
            leftMotorOutput[1] = -yTranslation;
            rightMotorOutput[0] = yTranslation;
            rightMotorOutput[1] = -yTranslation;
        }

        else {
            Arrays.fill(leftMotorOutput, 0);
            Arrays.fill(rightMotorOutput, 0);
        }

        this.leftRear.setPower(limit(leftMotorOutput[0]));
        this.leftFront.setPower(limit(leftMotorOutput[1]));
        this.rightRear.setPower(limit(rightMotorOutput[0]));
        this.rightFront.setPower(limit(rightMotorOutput[1]));
    }

//    public void getMotors () {
//        telemetry.addData("velocity",  "velocity %7d :%7d :%7d :%7d",
//                leftFront.getPower()
//                rightFront.getPower(),
//                leftRear.getPower(),
//                rightRear.getPower();
//    }

    public double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        return Math.max(value, -1.0);
    }

    protected double applyDeadBand(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }


}