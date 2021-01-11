package org.firstinspires.ftc.lib.kinematics;

import org.jetbrains.annotations.NotNull;

public class ChassisSpeed {

    public double vx;
    public double vy;
    public double omega;

    public ChassisSpeed() {}

    public ChassisSpeed(double vX, double vY, double omega) {
        this.vx = vX;
        this.vy = vY;
        this.omega = omega;
    }

    @Override
    public String toString() {
        return "ChassisSpeeds{" +
                "vx=" + vx +
                ", vy=" + vy +
                ", omega=" + omega +
                '}';
    }
}
