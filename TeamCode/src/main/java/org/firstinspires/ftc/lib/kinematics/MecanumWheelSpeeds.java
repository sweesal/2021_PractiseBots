package org.firstinspires.ftc.lib.kinematics;

public class MecanumWheelSpeeds {

    public double leftFrontMetersPerSecond;
    public double leftRearMetersPerSecond;
    public double rightFrontMetersPerSecond;
    public double rightRearMetersPerSecond;

    public MecanumWheelSpeeds() {
    }

    public MecanumWheelSpeeds(
            double leftFrontMetersPerSecond,
            double rightFrontMetersPerSecond,
            double leftRearMetersPerSecond,
            double rightRearMetersPerSecond) {
        this.leftFrontMetersPerSecond = leftFrontMetersPerSecond;
        this.rightFrontMetersPerSecond = rightFrontMetersPerSecond;
        this.leftRearMetersPerSecond = leftRearMetersPerSecond;
        this.rightRearMetersPerSecond = rightRearMetersPerSecond;
    }

}
