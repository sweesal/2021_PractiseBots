package org.firstinspires.ftc.lib.kinematics;


import org.ejml.simple.SimpleMatrix;

public class MecanumDriveKinematics {
    private final SimpleMatrix fwdKinematics;
    private final SimpleMatrix invKinematics;
    private SimpleMatrix wheelSpeedMatrix = new SimpleMatrix(4, 1); // 4*1 matrix
    private SimpleMatrix chassisSpeedMatrix = new SimpleMatrix(3, 1); // 3*1 matrix

    public MecanumDriveKinematics (double robotLength, double robotWidth) {

        double ROBOT_WIDTH_HALF = robotWidth / 2;
        double ROBOT_LENGTH_HALF = robotLength / 2;
        invKinematics = new SimpleMatrix(3, 3);
        setInvKinematics(ROBOT_LENGTH_HALF, ROBOT_WIDTH_HALF);
        fwdKinematics = invKinematics.pseudoInverse();

    }

    private void setInvKinematics (double halfLength, double halfWidth) {
        double frameParam = halfLength + halfWidth;
        fwdKinematics.setRow(0, 0, 1, -1, frameParam);
        fwdKinematics.setRow(1, 0, 1,  1, -frameParam);
        fwdKinematics.setRow(2, 0, 1, -1, -frameParam);
        fwdKinematics.setRow(3, 0, 1,  1, frameParam);
    }

    private void setInvKinematics (double halfLength, double halfWidth, double rollerAngle) {
        // When roller angle != 45 degrees.
    }

    public ChassisSpeed toChassisSpeed (MecanumWheelSpeeds wheelSpeeds){
        wheelSpeedMatrix.setColumn(0, 0,
                wheelSpeeds.rightFrontMetersPerSecond, wheelSpeeds.leftFrontMetersPerSecond,
                wheelSpeeds.leftRearMetersPerSecond, wheelSpeeds.rightRearMetersPerSecond); // Anticlockwise.
        chassisSpeedMatrix = fwdKinematics.mult(wheelSpeedMatrix);
        return new ChassisSpeed(
                chassisSpeedMatrix.get(0, 0),
                chassisSpeedMatrix.get(1, 0),
                chassisSpeedMatrix.get(2, 0)
        );
    }

    public MecanumWheelSpeeds toWheelSpeeds (ChassisSpeed chassisSpeed) {
        chassisSpeedMatrix.setColumn(0,0,
                chassisSpeed.vy,
                chassisSpeed.vx,
                chassisSpeed.omega);
        wheelSpeedMatrix = invKinematics.mult(chassisSpeedMatrix);
        return new MecanumWheelSpeeds(
                wheelSpeedMatrix.get(0, 0),
                wheelSpeedMatrix.get(1, 0),
                wheelSpeedMatrix.get(2, 0),
                wheelSpeedMatrix.get(3, 0)
        );
    }


}
