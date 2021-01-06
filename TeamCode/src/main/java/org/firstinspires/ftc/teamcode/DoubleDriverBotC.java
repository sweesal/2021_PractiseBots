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

import org.firstinspires.ftc.teamcode.robotC.RobotMapBotC;
import org.firstinspires.ftc.teamcode.robotC.subsystems.DriveTrainC;
import org.firstinspires.ftc.teamcode.robotC.subsystems.IntakeC;
import org.firstinspires.ftc.teamcode.robotC.subsystems.ShooterC;
import org.firstinspires.ftc.teamcode.robotC.subsystems.WobbleGoalMover;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Double Driver Mode C", group="C")
//@Disabled

public class DoubleDriverBotC extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotMapBotC robotMapBotC = new RobotMapBotC();
    private DriveTrainC driveTrain;
    private IntakeC intake;
    private ShooterC shooter;
    private WobbleGoalMover wobbleGoalMover;

    @Override
    public void init() {
        robotMapBotC.robotInit(hardwareMap);
        driveTrain = new DriveTrainC();
        intake = new IntakeC();
        shooter = new ShooterC();
        wobbleGoalMover = new WobbleGoalMover();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // DriveTrain.
        driveTrain.driveMecanum(
                -gamepad1.left_stick_y*0.99, -gamepad1.right_stick_x*0.99, -gamepad1.left_stick_x*0.99, gamepad1.x);

        // Superstructure
        intake.setIntake(gamepad2.x, gamepad2.b);
        shooter.setShooter(gamepad2.left_bumper);
        shooter.setTrigger(gamepad2.right_bumper);
        shooter.setElevator(gamepad2.right_stick_y*0.6, false, false);

        // Wobble goal mover.
        wobbleGoalMover.setIntake(gamepad2.y, gamepad2.a);

        // This is for showing the encoder & switch value of the elevator.
        telemetry.addData("Elevator Position", "%5.2f", shooter.getElevator());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    @Override
    public void stop() {

    }


}
