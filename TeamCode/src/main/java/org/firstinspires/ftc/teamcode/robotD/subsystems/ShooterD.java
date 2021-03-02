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

    private double currentStateStartTime = 0;

    private final DcMotor shooter = RobotMapBotD.shooter;
    private final Servo trigger = RobotMapBotD.trigger;

    private static boolean isShooting = false; //default state;
    private ElapsedTime shootingChangeTimer = new ElapsedTime();

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
        SHOOTER_STATE newState = shooterState;
        double currentTime = getTimer();
        double timeInState = currentTime - currentStateStartTime;

        // State transition
        switch (shooterState) {
            case IDLE:
                newState = handleIdleTransition(timeInState);
                break;
            case SPEEDING_UP:
                newState = handleSpeedingUpTransition (timeInState, shooterState);
                break;
            case FULL_POWER:
                newState = handleFullPowerTransition (timeInState, shooterState);
                break;
            case MANUAL:
                newState = handleManualTransition (); // btn ctrl
                break;
            default:
                System.out.println("Unexpected shooter state: " + shooterState);
                newState = shooterState;
                break;
        }

        if (newState != shooterState) {
            System.out.println(currentTime + ": Changed state: " + shooterState + " -> " + newState);
            shooterState = newState;
            currentStateStartTime = currentTime;
            timeInState = 0.0;
        }

        // State action
        switch (shooterState) {
            case IDLE:
                setShooterFree(0);
                break;
            case SPEEDING_UP:
                speedingUpShooter(timeInState);
                break;
            case FULL_POWER:
                setShooterFree(0.99);
                break;
            case MANUAL:
                break;
            default:
                System.out.println("Unexpected gear grabber system state: " + shooterState);
                break;
        }
    }

    private SHOOTER_STATE handleManualTransition() {
        return SHOOTER_STATE.MANUAL;
    }

    private SHOOTER_STATE handleIdleTransition(double timeInState) {
        return SHOOTER_STATE.IDLE;
    }

    private SHOOTER_STATE handleSpeedingUpTransition(double timeInState, SHOOTER_STATE shooterState) {
        switch (shooterState) {
            case IDLE:
            case SPEEDING_UP:
                return SHOOTER_STATE.SPEEDING_UP;
            case FULL_POWER:
                if (timeInState > kShootDelayTime)
                    return SHOOTER_STATE.FULL_POWER;
                return SHOOTER_STATE.SPEEDING_UP;
        }
        return SHOOTER_STATE.SPEEDING_UP;
    }

    private SHOOTER_STATE handleFullPowerTransition(double timeInState, SHOOTER_STATE shooterState) {
        switch (shooterState) {
            case IDLE:
                if (timeInState > kSpeedingUpDelayTime)
                    return SHOOTER_STATE.IDLE;
                return SHOOTER_STATE.FULL_POWER;
            case SPEEDING_UP:
                if (timeInState > kShootDelayTime)
                    return SHOOTER_STATE.FULL_POWER;
                return SHOOTER_STATE.SPEEDING_UP;
            case FULL_POWER:
                return SHOOTER_STATE.FULL_POWER;
            case MANUAL:
                return SHOOTER_STATE.MANUAL;
        }

        return SHOOTER_STATE.FULL_POWER;
    }


    private void speedingUpShooter(double timeInState) {
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
