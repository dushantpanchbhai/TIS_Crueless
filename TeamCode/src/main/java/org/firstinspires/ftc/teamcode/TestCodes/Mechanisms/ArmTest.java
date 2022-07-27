package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "arm test",group = "test")
public class ArmTest extends LinearOpMode {
    DcMotorEx extension_motor1, extension_motor2, turnTable, intake;
    Servo servo_dropper, servo_arm, left_intake, right_intake;

    @Override
    public void runOpMode() throws InterruptedException {
        servo_arm = hardwareMap.get(Servo.class, "Arm");
        servo_dropper = hardwareMap.get(Servo.class, "Dropper");
        extension_motor1 = hardwareMap.get(DcMotorEx.class, "extension1");
        extension_motor2 = hardwareMap.get(DcMotorEx.class, "extension2");

        extension_motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension_motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extension_motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo_dropper.setPosition(ArmConstants.servo_dropper_pickup_pos);
        servo_arm.setPosition(ArmConstants.servo_arm_pickup_pos);

        int pos = 0;

        waitForStart();

        extension_motor1.setTargetPosition(0);
        extension_motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor1.setVelocity(500);
        extension_motor2.setTargetPosition(0);
        extension_motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor2.setVelocity(500);

        while (opModeIsActive()) {

            if(gamepad1.y)
            {
                pos = ArmConstants.extension_max_pos;
                servo_dropper.setPosition(ArmConstants.servo_dropper_drop_pos);
                servo_arm.setPosition(ArmConstants.servo_arm_drop_pos_wait);
                extendTo(pos);
            }
            else if(gamepad1.b)
            {
                pos = ArmConstants.extension_low_pos;
                servo_dropper.setPosition(ArmConstants.servo_dropper_pickup_pos);
                servo_arm.setPosition(ArmConstants.servo_arm_pickup_pos);
                extendTo(pos);
            }

            if(gamepad1.x)
            {
                pos = pos + 10;
                extendTo(pos);
            }

            if(gamepad1.a)
            {
                pos = pos - 10;
                extendTo(pos);
            }

            telemetry.addData("pos is ",pos);
            telemetry.addData("motor 1 position",extension_motor1.getCurrentPosition());
            telemetry.addData("motor 2 posiiton",extension_motor2.getCurrentPosition());
            telemetry.update();
        }
    }

    public void extendTo(int pos){
        extension_motor1.setTargetPosition(pos);
        extension_motor2.setTargetPosition(-pos);

        extension_motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if( Math.abs(extension_motor2.getCurrentPosition() - extension_motor2.getTargetPosition()) < 1){
            extension_motor2.setVelocity(0);
        }
        else{
            extension_motor2.setVelocity(1500);
        }
        if(Math.abs(extension_motor1.getCurrentPosition() - extension_motor1.getTargetPosition()) < 1){
            extension_motor1.setVelocity(0);
        }
        else{
            extension_motor1.setVelocity(1500);
        }
    }
}
