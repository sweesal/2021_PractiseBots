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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotC.subsystems.DriveTrainC;
import org.firstinspires.ftc.teamcode.robotC.subsystems.IntakeC;
import org.firstinspires.ftc.teamcode.robotC.subsystems.ShooterC;
import org.firstinspires.ftc.teamcode.robotC.RobotMapBotC;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="单人模式 C车", group="C")
//@Disabled

public class SingleDriverBotC extends OpMode {
    RobotMapBotC robotMapBotC = new RobotMapBotC();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeStamp = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private DriveTrainC driveTrain;
    private IntakeC intake;
    private ShooterC shooter;
    private static final int PERIOD = 500;
    private static double prevTimeStamp = -1;
    private static double currentTimeStamp = 0;
    private int periodCount = 0;

    @Override
    public void init() {
        robotMapBotC.robotInit(hardwareMap);
        driveTrain = new DriveTrainC();
        intake = new IntakeC();
        shooter = new ShooterC();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // DriveTrain.
        driveTrain.driveMecanum(
                -gamepad1.left_stick_y*0.99, -gamepad1.right_stick_x*0.99, -gamepad1.left_stick_x*0.99, gamepad1.x);

        // Superstructure
        intake.setIntake(gamepad1.x, gamepad1.b);
        shooter.setShooter(gamepad1.left_bumper);
        shooter.setTrigger(gamepad1.right_bumper);
        shooter.setElevator(gamepad1.right_stick_y*0.6, false, shooter.getSwitchLower());

        periodTest();

        telemetry.addData("Status", "Period Time: " + timeStamp.toString());

        telemetry.addData("Period Number", periodCount);
        // This is for showing the encoder & switch value of the elevator.
        telemetry.addData("Elevator Position", "%5.2f", shooter.getElevator());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    @Override
    public void stop() {

    }

    public void periodTest () {
        currentTimeStamp = timeStamp.milliseconds();
        if (currentTimeStamp - prevTimeStamp > PERIOD) {
            periodCount++;
            prevTimeStamp = currentTimeStamp;
        }
    }

}
