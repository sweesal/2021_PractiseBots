package org.firstinspires.ftc.teamcode.robotB.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotB.ConstantsB;
import org.firstinspires.ftc.teamcode.robotB.RobotMapBotB;

public class DriveTrainB {

    private final DcMotor leftFront = RobotMapBotB.leftFront;
    private final DcMotor leftRear = RobotMapBotB.leftRear;
    private final DcMotor rightFront = RobotMapBotB.rightFront;
    private final DcMotor rightRear = RobotMapBotB.rightRear;
    private BNO055IMU imu = RobotMapBotB.imu;

    private ElapsedTime timeStamp = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private static double prevTimeStamp = -1;
    private static final int PERIOD = 50;

    private MotorConfigurationType motorConfig = MotorConfigurationType.getMotorType(RevRobotics20HdHexMotor.class);

    public DriveTrainB() {
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

    public double getYaw(){
        return 180 / Math.PI * imu.getAngularOrientation().firstAngle;
    }

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

    public void stopMoors () {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    private double encoderTicksToMeters (double encoderInput) {
        return encoderInput * Math.PI * ConstantsB.WHEEL_DIAMETER / motorConfig.getTicksPerRev();
    }

    public double getAverageEncoderPosition () {
        double encoderPositionTotal =
                leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                        + rightFront.getCurrentPosition() + rightRear.getCurrentPosition();
        return encoderTicksToMeters(encoderPositionTotal / 4);
    }

    private void runWithoutEncoder () {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopAndResetEncoder () {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runToPosition () {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void velocityToMeters () {

    }

    private void setEncoderPosition (double position) {

    }

    private void getTrapezoidVelocityDuration (double position, double velocity) {

    }

    public void moveToEncoderPosition (double position, double velocity) {

    }

    public void turnTo (int degrees) {
        double kp = 0.03, kf = -0.1;
        double currentAngle = autoRobot.readHeading();
        double error = setPoint - currentAngle;
        double output = kp * error + kf;
        if(output > 0.8)        output = 0.8;
        else if(output < -0.8)  output = -0.8;
    }

}