package org.firstinspires.ftc.lib.odometry;

import org.firstinspires.ftc.lib.geometry.Pose2d;
import org.firstinspires.ftc.lib.geometry.Rotation2d;
import org.firstinspires.ftc.lib.kinematics.MecanumDriveKinematics;

public class MecanumOdometry {
    private MecanumDriveKinematics kinematics;
    private Pose2d pose;

    private Rotation2d gyroOffset;
    private Rotation2d prevAngle;

    public MecanumOdometry (
            MecanumDriveKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPoseMeters) {
        this.kinematics = kinematics;
        this.pose = initialPoseMeters;
        this.gyroOffset = pose.getRotation().rotateBy(gyroAngle.inverse());
        this.prevAngle = initialPoseMeters.getRotation();
    }

    public MecanumOdometry (
            MecanumDriveKinematics kinematics, Rotation2d gyroAngle) {
        this(kinematics, gyroAngle, new Pose2d());
    }

    public void reset (Pose2d poseMeters, Rotation2d gyroAngle) {
        this.pose = poseMeters;
        this.prevAngle = poseMeters.getRotation();
        gyroOffset = pose.getRotation().rotateBy(gyroAngle.inverse());
    }

    public Pose2d getPose () {
        return pose;
    }



}
