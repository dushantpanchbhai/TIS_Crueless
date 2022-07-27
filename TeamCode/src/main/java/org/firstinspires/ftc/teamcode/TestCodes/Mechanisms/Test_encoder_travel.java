package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@Disabled
@TeleOp(name="Test Encoder Count", group = "Test")
public class Test_encoder_travel extends LinearOpMode{

    DcMotorEx motor;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Motor position is ",motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
