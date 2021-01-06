package org.firstinspires.ftc.lib.kinematics;

public class DifferentialDriveKinematics {
    public final double trackWidth;

    public DifferentialDriveKinematics (double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public ChassisSpeeds forwardKinematics () {
        return new ChassisSpeeds();
    }
}
