package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Config;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator {
    HardwareMap hwMap = null;

    //Elevator Parameters
    DcMotor elevator = null;
    final int LIFT_COUNTS_PER_UPDOWN_EFFORT = 50;
    final double LIFT_POWER = 1.0;

    //Grabber Parameters
    Servo grabber = null;
    final double GRABBER_CLOSED_POSITION = 0.0;
    final double GRABBER_OPENED_POSITION = 0.8 ;

    public void init(HardwareMap Map, Config config) {
        hwMap = Map;

        elevator = hwMap.get(DcMotor.class, "elevator");
        grabber = hwMap.get(Servo.class, "grabber");

        elevator.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grabber.setDirection(Servo.Direction.FORWARD);
    }

    public void setLiftZeroPosition() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void moveUp() {
        int newTarget;
        newTarget  = elevator.getCurrentPosition() + LIFT_COUNTS_PER_UPDOWN_EFFORT;
        elevator.setPower(LIFT_POWER);
        elevator.setTargetPosition(newTarget);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveDown() {
        int newTarget;
        newTarget  = elevator.getCurrentPosition() - LIFT_COUNTS_PER_UPDOWN_EFFORT;
        elevator.setPower(LIFT_POWER);
        elevator.setTargetPosition(newTarget);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop() {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        elevator.setPower(0.0);
    }

    public void closeGrabber() {
        grabber.setPosition(GRABBER_CLOSED_POSITION);
    }

    public void openGrabber() {
        grabber.setPosition(GRABBER_OPENED_POSITION);
    }
}