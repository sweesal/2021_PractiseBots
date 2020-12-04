package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

class RobotMapBotB {
    /* Public OpMode members. */
    //motor
    public DcMotorEx leftFront  = null;
    public DcMotorEx leftRear   = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear  = null;
    public DcMotorEx shooter    = null;
    public DcMotorEx plateLift = null;
    public DcMotorEx intake     = null;
    public DcMotorEx ringStack = null;
    //Servo
    public Servo ringPush     = null;
    //    public CRServo ringStack    = null;
    public Servo angleChanger   = null;
    //sensor
    public DigitalChannel downLimit = null;
    public DigitalChannel upLimit   = null;
//    public RevColorSensorV3 lineSensor = null;

    public static final double MID_SERVO       =  0.55 ;
    public static final double RING_PUSH_INIT_POSITION = 0.2;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RobotMapBotB(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class,"leftfront");
        leftRear = hwMap.get(DcMotorEx.class,"leftrear");
        rightFront = hwMap.get(DcMotorEx.class,"rightfront");
        rightRear = hwMap.get(DcMotorEx.class,"rightrear");
        shooter = hwMap.get(DcMotorEx.class,"shooter");
        plateLift = hwMap.get(DcMotorEx.class,"plateLift");
        intake = hwMap.get(DcMotorEx.class,"intake");
        ringStack = hwMap.get(DcMotorEx.class,"ringstack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        plateLift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ringStack.setDirection(DcMotorSimple.Direction.REVERSE);

        plateLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
        shooter.setPower(0.0);
        plateLift.setPower(0.0);
        intake.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plateLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        ringPush = hwMap.get(Servo.class,"ringpush");
//        ringStack = hwMap.get(CRServo.class,"ringStack");
        angleChanger = hwMap.get(Servo.class, "anglechanger");
        ringPush.setDirection(Servo.Direction.FORWARD);
        ringStack.setDirection(DcMotorSimple.Direction.REVERSE);
        angleChanger.setDirection(Servo.Direction.REVERSE);
        ringPush.setPosition(RING_PUSH_INIT_POSITION);
        ringStack.setPower(0.0);
        angleChanger.setPosition(MID_SERVO);

        //initialize sensors
        downLimit = hwMap.get(DigitalChannel.class,"downlimit");
        upLimit = hwMap.get(DigitalChannel.class,"uplimit");
//        lineSensor = hwMap.get(RevColorSensorV3.class,"lineSensor");
        downLimit.setMode(DigitalChannel.Mode.INPUT);
        upLimit.setMode(DigitalChannel.Mode.INPUT);
    }
}
