/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import java.util.concurrent.TimeUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Single Driver Mode", group="Iterative Opmode")
//@Disabled

// Plz ignore this disabled class.

public class TeleOpMode_Iterative extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    RobotMap robotMap = new RobotMap();
    private DriveTrain driveTrain;
    private Intake intake;
    private Shooter shooter;

    private boolean triggerFlag = false;
    private double triggerTime = 0;

    @Override
    public void init() {
        robotMap.robotInit(hardwareMap);
        driveTrain = new DriveTrain();
        intake = new Intake();
        shooter = new Shooter();
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void loop() {

        // DriveTrain.
        driveTrain.driveMecanum(
                gamepad1.left_stick_y*0.7, gamepad1.right_stick_x*0.7, gamepad1.left_stick_x*0.7, gamepad1.x);

        // Superstructure
        intake.setIntake(gamepad1.left_bumper);
        shooter.setShooter(gamepad1.right_bumper);
        shooter.ctrlSlope(gamepad1.right_trigger);
        shooter.setTrigger(gamepad1.left_trigger > 0.5);

        //shooter.setElevator(-gamepad1.right_stick_y, !shooter.getSwitch(), shooter.getElevator() < 0);
        shooter.elevatorMove(gamepad1.y, gamepad1.a, !shooter.getSwitch(), shooter.getElevator() < 0);

        // This is for showing the encoder & switch value of the elevator.
        telemetry.addData("Elevator Position", "%5.2f", shooter.getElevator());
        telemetry.addData("Limit Switch", shooter.getSwitch());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    public void triggerMove () {
        double duration = 1000;//ms
        boolean triggerTempA = false;
        boolean triggerTempB = false;
        boolean finalState = false;
        boolean triggerCmd = false;

        triggerCmd = finalState;
        shooter.setTrigger(triggerCmd);

//        double duration = 5000;//ms
//        if (!shooter.getSwitch()) {
//            triggerFlag = true;
//        }
//        if (triggerFlag)
//            triggerTime = runtime.now(TimeUnit.MILLISECONDS) + duration;
//        if (triggerTime > runtime.now(TimeUnit.MILLISECONDS))
//            shooter.setTrigger(0.175);
//        else {
//            shooter.setTrigger(0.6);
//            triggerFlag = false;
//        }
//        triggerFlag = true;
//        double duration = 5000;//ms
//        triggerTime = runtime.now(TimeUnit.MILLISECONDS) + duration;
//        if (triggerTime - runtime.now(TimeUnit.MILLISECONDS) > 0) {
//            shooter.setTrigger(0.175);
//        } else {
//            triggerFlag = false;
//            shooter.setTrigger(0.6);
//        }

    }

}
