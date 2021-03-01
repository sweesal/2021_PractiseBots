package org.firstinspires.ftc.teamcode.robotD.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotD.RobotMapBotD;

public class ShooterD {

    private ElapsedTime shootingTimer = new ElapsedTime();

    private double kSpeedingUpDelayTime = 3;
    private double kShootDelayTime = 1;

    private final DcMotor shooter = RobotMapBotD.shooter;
    private final Servo trigger = RobotMapBotD.trigger;

    private static boolean isShooting = false; //default state;

    private enum SHOOTER_STATE {
        IDLE, SPEEDING_UP, FULL_POWER, MANUAL
    }


    private SHOOTER_STATE shooterState = SHOOTER_STATE.IDLE;

    public ShooterD() {
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        trigger.setDirection(Servo.Direction.REVERSE);
    }

    public void setTimedAutoShoot () {

    }

    private void launchRings () {

    }

    private void updateShooter (double timeStamp) {


        switch (shooterState) {
            case IDLE:
                setShooterFree(0);
            case SPEEDING_UP:

            case FULL_POWER:


        }
    }

    private double getTimer () {
        return shootingTimer.milliseconds();
    }

    private void resetTimer () {
        shootingTimer.reset();
    }

    public void setShooterFree (double input) {
        double output = Range.clip(input, -0.99, 0.99);
        shooter.setPower(output);
    }

    public void setShooter (boolean isBtnPressed) {
        if (shooterState == SHOOTER_STATE.MANUAL) {
            if (isBtnPressed)
                isShooting = !isShooting;
            if (isShooting)
                setShooterFree(0.95);//shooting power
            else
                setShooterFree(0);//stop shooting
        }
    }


    private void setTriggerFree (double inputPos) {
        double pos = Range.clip(inputPos, -0.95, 0.95);
        trigger.setPosition(pos);
    }

    public void setTrigger (boolean isBtnPressed) {
        if (isBtnPressed)
            setTriggerFree(0.175);
        else
            setTriggerFree(0.475);
    }

    public SHOOTER_STATE getShooterState() {
        return shooterState;
    }

    public void setShooterState(SHOOTER_STATE shooterState) {
        this.shooterState = shooterState;
    }


}
