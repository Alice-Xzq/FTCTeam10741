/*
rleyvacortes22 is big brain 这是大脑时间
rmehta22
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "TeleopOnePlayer", group = "Linear Opmode")
//@Disabled
public class Teleop_OnePlayer extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    DcMotor elevator;
    Servo  leftServo;
    Servo rightServo;

    @Override
    public void runOpMode() {

        double speedAdjust = 7.0;
        double servoPos1 = 0.0;

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftServo.setPosition(0.0);
        rightServo.setPosition(0.0);

        runtime.reset();



        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_left) {
                speedAdjust -= 0.5;
            }
            if(gamepad1.dpad_right) {
                speedAdjust += 0.5;
            }

            //elevator
            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevator.setPower(gamepad1.right_trigger-gamepad2.left_trigger);

            //Gripper Position
            if (gamepad1.right_bumper) {
                servoPos1 += 0.05;

                if (servoPos1 >= 0.5) {
                    servoPos1 = 0.9;
                }
            }
            else if (gamepad1.left_bumper) {
                servoPos1 -= 0.05;

                if (servoPos1 <= 0.5) {
                    servoPos1 = 0.0;
                }
            }

            leftServo.setPosition(servoPos1);

            // Mecanum Driving Code

            leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*(-speedAdjust/10));

            rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*(-speedAdjust/10));

            leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)*(-speedAdjust/10));

            rightFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)*(-speedAdjust/10));


            // Pace this loop so jaw action is reasonable speed.
            sleep(20);
            telemetry.update();
        }
    }
}
