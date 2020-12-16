package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Double-Driver Mode Bt", group="Iterative Opmode")
//@Disabled

public class DoubleDriverBotBt extends OpMode{
    private static final double SHOOTER_VELOCITY = 1800;

    //private HardwareRobot hardwareRobot = new HardwareRobot();
    private final RobotMapBotBt hardwareRobot = new RobotMapBotBt();
    private double m_shooterAngle = 0.55;
    private boolean isShoot = false;
    private int shoottime = 0;
    private boolean pushRing = false;
    private ElapsedTime keyDelay = new ElapsedTime();
    private ElapsedTime ringPushDelay = new ElapsedTime();

    @Override
    public void init() {
        //telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        hardwareRobot.init(hardwareMap);
        telemetry.addData("Status","initialized.");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        keyDelay.reset();
        keyDelay.reset();
        ringPushDelay.reset();
    }

    @Override
    public void loop() {
        //drive base
//        __________________________________________________________________________
        double drive = -gamepad1.left_stick_y;
        double turn =  gamepad1.right_stick_x;
        double translation = gamepad1.left_stick_x;
        if(Math.abs(drive) < 0.05)
        {
            drive = 0.0;
        }
        if(Math.abs(turn) < 0.05)
        {
            turn = 0.0;
        }
        if(Math.abs(translation) < 0.05)
        {
            translation = 0.0;
        }

        double[] speed ={
                drive + turn + translation,
                drive + turn - translation,
                drive - turn - translation,
                drive - turn + translation};
        double max = 1.0;
        for(double x : speed){
            if(Math.abs(x) > max)   max = Math.abs(x);
        }
        // if power value more than 1.0, dived by max;
        if(max > 1.0)
        {
            for(int i = 0; i< speed.length; i++)
            {
                speed[i] /= max;
            }
        }
        driveBaseMove(speed[0],speed[1],speed[2],speed[3]);
        telemetry.addData("leftFrontVel",hardwareRobot.leftFront.getVelocity());
        telemetry.addData("leftRearVel",hardwareRobot.leftRear.getVelocity());
        telemetry.addData("rightFrontVel",hardwareRobot.rightFront.getVelocity());
        telemetry.addData("rightRearVel",hardwareRobot.rightRear.getVelocity());

        //intake
//        ____________________________________________________________________

        if(gamepad2.x) {
            if(hardwareRobot.downLimit.getState()) {
                hardwareRobot.plateLift.setPower(-0.5);
            }
            else {
                hardwareRobot.plateLift.setPower(0.0);
                hardwareRobot.intake.setPower(1.0);
                hardwareRobot.ringStack.setPower(1.0);
            }

        } else{
            if(gamepad2.b) {
                hardwareRobot.intake.setPower(-1.0);
            } else {
                hardwareRobot.intake.setPower(0.0);
            }
            hardwareRobot.ringStack.setPower(0.0);
        }

        //angleChanger
        if(gamepad2.dpad_up && keyDelay.seconds() > 0.2)
        {
            m_shooterAngle +=0.025;
            keyDelay.reset();
        }
        else if(gamepad2.dpad_down && keyDelay.seconds() >0.2)
        {
            m_shooterAngle -=0.025;
            keyDelay.reset();
        }
        if(m_shooterAngle > 0.75)
        {
            m_shooterAngle = 0.75;
        }
        if(m_shooterAngle < 0.55)
        {
            m_shooterAngle = 0.55;
        }
        hardwareRobot.angleChanger.setPosition(m_shooterAngle);
        telemetry.addData("Shooter Angle","%2f",m_shooterAngle);

        //plate lift
//        __________________________________________________________________
        telemetry.addData("upLimit",hardwareRobot.upLimit.getState());
        telemetry.addData("upLimit",hardwareRobot.downLimit.getState());
        double liftPower = -gamepad2.right_stick_y * 0.6;
        if(!hardwareRobot.upLimit.getState() && liftPower > 0.0)
        {
            hardwareRobot.plateLift.setPower(0.0);
        }
        else if(!hardwareRobot.downLimit.getState() && liftPower < 0.0)
        {
            hardwareRobot.plateLift.setPower(0.0);
        }
        else
        {
            hardwareRobot.plateLift.setPower(liftPower);
        }
//        ring push
//        ________________________________________________________________
        if(gamepad2.right_bumper && keyDelay.seconds() > 0.3)
        {
            pushRing = true;
            keyDelay.reset();
            ringPushDelay.reset();
        }
        if(pushRing)
        {
            hardwareRobot.ringPush.setPosition(0.7);
            if(ringPushDelay.seconds() > 1.0)
            {
                hardwareRobot.ringPush.setPosition(0.2);
                pushRing = false;
            }
        }


        //shooter
//        _________________________________________________________________
        if(gamepad2.left_bumper && keyDelay.seconds() >0.2)
        {
            isShoot = !isShoot;
            keyDelay.reset();
        }
        if(isShoot)
        {
            hardwareRobot.shooter.setVelocity(SHOOTER_VELOCITY);
        }
        else
        {
            hardwareRobot.shooter.setMotorDisable();
        }
        telemetry.addData("shooter velocity", "%2f",
                hardwareRobot.shooter.getVelocity());
//        if(gamepad1.left_bumper && shootDelay.seconds() > 0.5)
//        {
//            isShoot = !isShoot;
//            shootDelay.reset();
//        }
//
//        if(isShoot)
//        {
//            hardwareRobot.shooter.setVelocity(100);
//
//        }
//        else
//        {
//            hardwareRobot.shooter.setMotorDisable();
//            hardwareRobot.ringPush.setPosition(0.5);
//        }

    }

    @Override
    public void stop() {

    }
    public void driveBaseMove(double lf,double lr,double rf,double rr) {
        hardwareRobot.leftFront.setPower(lf);
        hardwareRobot.leftRear.setPower(lr);
        hardwareRobot.rightFront.setPower(rf);
        hardwareRobot.rightRear.setPower(rr);
    }
}
