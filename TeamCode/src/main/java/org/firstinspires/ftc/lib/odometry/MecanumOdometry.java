package org.firstinspires.ftc.lib.odometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.lib.geometry.Pose2d;
import org.firstinspires.ftc.lib.geometry.Rotation2d;
import org.firstinspires.ftc.lib.geometry.Translation2d;
import org.firstinspires.ftc.lib.geometry.Twist2d;
import org.firstinspires.ftc.lib.kinematics.ChassisSpeed;
import org.firstinspires.ftc.lib.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.kinematics.MecanumWheelSpeeds;

public class MecanumOdometry {
    private static MecanumDriveKinematics kinematics;
    private static Pose2d pose;

    private Rotation2d gyroOffset;
    private Rotation2d prevAngle;

    private static final ElapsedTime timeStamp = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private static double prevTimeStamp = -1;
    private static final int PERIOD = 20;

    public MecanumOdometry (
            MecanumDriveKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPoseMeters) {
        MecanumOdometry.kinematics = kinematics;
        pose = initialPoseMeters;
        this.gyroOffset = pose.getRotation().rotateBy(gyroAngle.inverse());
        this.prevAngle = initialPoseMeters.getRotation();
    }

    public MecanumOdometry (
            MecanumDriveKinematics kinematics, Rotation2d gyroAngle) {
        this(kinematics, gyroAngle, new Pose2d());
    }

    public void reset (Pose2d poseMeters, Rotation2d gyroAngle) {
        pose = poseMeters;
        this.prevAngle = poseMeters.getRotation();
        gyroOffset = pose.getRotation().rotateBy(gyroAngle.inverse());
    }

    public Pose2d getPose () {
        return pose;
    }

    public Pose2d updatePose (Rotation2d gyroAngle, MecanumWheelSpeeds wheelSpeeds) {
        prevTimeStamp = timeStamp.milliseconds() - prevTimeStamp > PERIOD ? timeStamp.milliseconds() : 0.0;

        Rotation2d angle = gyroAngle.rotateBy(gyroOffset);
        ChassisSpeed chassisSpeed = kinematics.toChassisSpeed(wheelSpeeds);
        Pose2d newPose = Pose2d.exp(new Twist2d(
                chassisSpeed.vx * PERIOD,
                chassisSpeed.vy * PERIOD,
                angle.getDegrees() - prevAngle.getDegrees())
        );
        prevAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);
        return pose;
    }




}
