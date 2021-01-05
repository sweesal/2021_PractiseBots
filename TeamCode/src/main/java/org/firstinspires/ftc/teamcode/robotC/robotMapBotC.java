package org.firstinspires.ftc.teamcode.robotC;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMapBotC {

    public HardwareMap hardwareMap;

    public static DcMotor leftFront = null;
    public static DcMotor leftRear = null;
    public static DcMotor rightFront = null;
    public static DcMotor rightRear = null;
    public static DcMotor intake = null;
    public static DcMotor shooter = null;
    public static DcMotor elevator = null;
    public static Servo trigger = null;

    public void robotInit(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotor.class,"leftfront");
        leftRear = hardwareMap.get(DcMotor.class,"leftrear");
        rightFront = hardwareMap.get(DcMotor.class,"rightfront");
        rightRear = hardwareMap.get(DcMotor.class,"rightrear");
        intake = hardwareMap.get(DcMotor.class,"intake");
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        elevator = hardwareMap.get(DcMotor.class,"elevator");
        trigger = hardwareMap.get(Servo.class,"trigger");
    }

    public void initDcMotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
