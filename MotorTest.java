package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name= "TestDrive", group = "Linear Opmode")

public class MotorTest extends LinearOpMode{
    HardwareMap hwMap =  null;
    DcMotor motor;

//    @Override
//    public void init(HardwareMap ahwMap) throws Exception {
//        // Save reference to Hardware map
//        hwMap = hardwareMap;
//        if(ahwMap==null) throw(new Exception("Hardware Map is null"));
//        motor = hwMap.get(DcMotor.class, "test");
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor.setPower(0);
//    }

    @Override
    public void runOpMode(){
        hwMap = hardwareMap;
        motor = hwMap.get(DcMotor.class, "test");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);

        while(opModeIsActive()) {
            double power = 1.0;
            motor.setPower(power);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", power);
            telemetry.update();
        }
    }
}
