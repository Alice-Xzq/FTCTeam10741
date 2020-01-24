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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryWrapper;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "TeleOp STREAM-A",group = "Final")
//@Disabled
public class TeleOpStreamA extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle

    static boolean sucking = false;
    static boolean pushing = false;
    static boolean clawStateOpened = false;

    private Config config = new Config(Config.configFile);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private ButtonHelper helper;
    private ButtonHelper helper2;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        helper = new ButtonHelper(gamepad1);
        helper2 = new ButtonHelper(gamepad2);
        Elevator elevator = new Elevator();
        DriveTrain driveTrain = new DriveTrain();

        driveTrain.init(hardwareMap,config);
        elevator.init(hardwareMap,config);

        elevator.setLiftZeroPosition();

        // Wait for the start button
        telemetry.addData(">>>", "Press to start TeleOp Mode" );
        telemetry.update();

        TelemetryWrapper.init(telemetry,11);
        elevator.closeGrabber();

        waitForStart();
        runtime.reset();

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            helper.update();
            helper2.update();

            double drivey =  -gamepad1.left_stick_y;
            double drivex =  -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;

            if (Math.abs(drivey) > 0.01 || Math.abs(drivex) > 0.01 || Math.abs(turn) > 0.01) {
                driveTrain.move(drivex, drivey, turn);
            } else {
                drivey = -gamepad2.left_stick_y * 0.25;
                drivex = -gamepad2.left_stick_x * 0.25;
                turn = gamepad2.right_stick_x * 0.25;
                driveTrain.move(drivex, drivey, turn);
            }

            if(gamepad1.dpad_up) {
                elevator.moveUp();
            }
            else if(gamepad1.dpad_down) {
                elevator.moveDown();
            }

            if(helper.pressing(ButtonHelper.x)) {
                clawStateOpened =! clawStateOpened;
            }

            if(clawStateOpened) {
                elevator.openGrabber();
            }
            else {
                elevator.closeGrabber();
            }

            // Show the elapsed game time and wheel power.
            TelemetryWrapper.setLine(0, "TeleOpMode v1.0" );
            TelemetryWrapper.setLine(1,"motors: drivex: " + drivex +"drivey: " + drivey+" turn: "+turn);
            TelemetryWrapper.setLine(2, "servoStatus: " + clawStateOpened);
            TelemetryWrapper.setLine(3,"Elevator status: ");

            idle();
        }

        telemetry.addData(">", "Done");
        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
