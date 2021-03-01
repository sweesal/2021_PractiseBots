package org.firstinspires.ftc.teamcode.robotD.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotD.RobotMapBotD;

import java.util.Arrays;

public class DriveTrainD {

    private final DcMotor leftFront = RobotMapBotD.leftFront;
    private final DcMotor leftRear = RobotMapBotD.leftRear;
    private final DcMotor rightFront = RobotMapBotD.rightFront;
    private final DcMotor rightRear = RobotMapBotD.rightRear;

    public DriveTrainD() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveMecanum(double xSpeed, double zRotation, double yTranslation, boolean isPowerHalfed) {
        double drive  = -limit(xSpeed);
        double strafe = limit(yTranslation);
        double twist  = limit(zRotation);

        if(isPowerHalfed) {
            drive = 0.5 * drive;
            strafe = 0.5 * strafe;
            twist = 0.5 * twist;
        }

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);

        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftRear.setPower(speeds[2]);
        rightRear.setPower(speeds[3]);
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
        } else if (zRotation != 0.0) {
            leftMotorOutput [0] = -zRotation;
            leftMotorOutput [1] = -zRotation;
            rightMotorOutput [0] = -zRotation;
            rightMotorOutput [1] = -zRotation;
        } else if (yTranslation != 0.0) {
            leftMotorOutput[0] = yTranslation;
            leftMotorOutput[1] = -yTranslation;
            rightMotorOutput[0] = yTranslation;
            rightMotorOutput[1] = -yTranslation;
        } else {
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