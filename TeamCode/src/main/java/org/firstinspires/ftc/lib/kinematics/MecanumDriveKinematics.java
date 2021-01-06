package org.firstinspires.ftc.lib.kinematics;


import org.ejml.simple.SimpleMatrix;

public class MecanumDriveKinematics {
    private final double ROBOT_WIDTH_HALF = 20; // Default value.
    private final double ROBOT_LENGTH_HALF = 20;
    private final double WHEEL_ROLLER_ANGLE = 45; // Default value, which is pi/4.
    private static ChassisSpeeds cSpeedToConvert;

    private SimpleMatrix fwdKinematics = new SimpleMatrix(3, 3);
    private SimpleMatrix invKinematics = new SimpleMatrix(3, 3);
    private SimpleMatrix wheelSpeeds = new SimpleMatrix(4, 1);
    private SimpleMatrix chassisSpeeds = new SimpleMatrix(3, 1);

    public MecanumDriveKinematics () {
        chassisSpeeds.setColumn(1, 1,
                cSpeedToConvert.vy, cSpeedToConvert.vx, cSpeedToConvert.omega);
        invKinematics = fwdKinematics.pseudoInverse();
    }

    private void getFwdKinematics () {
        double frameParam = ROBOT_LENGTH_HALF + ROBOT_WIDTH_HALF;
        if (WHEEL_ROLLER_ANGLE == 45) {
            fwdKinematics.setRow(1, 1, 1, -1, frameParam);
            fwdKinematics.setRow(2, 1, 1,  1, -frameParam);
            fwdKinematics.setRow(3, 1, 1, -1, -frameParam);
            fwdKinematics.setRow(4, 1, 1,  1, frameParam);
            wheelSpeeds = fwdKinematics.mult(chassisSpeeds);
        }
    }
}
