/*
rleyvacortes22 is big brain 这是大脑时间
rmehta22
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name= "Teleop", group = "Linear Opmode")


public class Teleop extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
//    Hardware robot = new Hardware();
    public DcMotor leftBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;
    public DcMotor elevator = null;
    public Servo claw = null;

    @Override
    public void runOpMode() {

        HardwareMap hwMap = hardwareMap;

        leftBack = hwMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = hwMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator = hwMap.get(DcMotor.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set all motors to power 0.0
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        elevator.setPower(0);

        // Define and initialize ALL installed servos.
        claw = hwMap.get(Servo.class, "claw");

        double speedAdjust = 7.0;
        double servoPos1 = 0.0;

        claw.setPosition(servoPos1);

        runtime.reset();

        waitForStart();

        while(opModeIsActive()){
            double magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double fld = magnitude * Math.cos(robotAngle) + rightX;
            final double frd = magnitude * Math.sin(robotAngle) - rightX;
            final double bld = magnitude * Math.sin(robotAngle) + rightX;
            final double brd = magnitude * Math.cos(robotAngle) - rightX;

            if(gamepad1.dpad_left) {
                speedAdjust -= 0.5;
            }
            if(gamepad1.dpad_right) {
                speedAdjust += 0.5;
            }

            //claw Position
            if (gamepad1.left_bumper) {
                servoPos1 += 0.05;

                if (servoPos1 >= 0.5) {
                    servoPos1 = 0.9;
                }
            }
            else if (gamepad1.right_bumper) {
                servoPos1 -= 0.05;

                if (servoPos1 <= 0.5) {
                    servoPos1 = 0.0;
                }
            }

            claw.setPosition(servoPos1);
            telemetry.addData("Servo Position", claw.getPosition());

            // Mecanum Driving Code

            leftFront.setPower(fld * (speedAdjust));
            rightFront.setPower(frd * (speedAdjust));
            leftBack.setPower(bld * (speedAdjust));
            rightBack.setPower(brd * (speedAdjust));

            //elevator
//            robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elevator.setPower(Range.clip(gamepad1.right_trigger-gamepad1.left_trigger,-1.0,1.0));

            // Pace this loop so jaw action is reasonable speed.
            sleep(20);
            telemetry.update();
        }
    }
}
