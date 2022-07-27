package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "servo test",group = "servo")
public class servoTEst extends LinearOpMode {

    DcMotorEx extension_motor1, extension_motor2, turnTable, intake;
    Servo servo_dropper, servo_arm, left_intake, right_intake;
    RevColorSensorV3 left_intake_sensor,  right_intake_sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        servo_arm = hardwareMap.get(Servo.class, "Arm");
        servo_dropper = hardwareMap.get(Servo.class, "Dropper");

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                servo_dropper.setPosition(ArmConstants.servo_dropper_pickup_pos);
                servo_arm.setPosition(ArmConstants.servo_arm_pickup_pos);
            }
            else if(gamepad1.dpad_right)
            {
                servo_dropper.setPosition(ArmConstants.servo_dropper_mid);
                servo_arm.setPosition(ArmConstants.servo_arm_mid);
            }
            else if(gamepad1.dpad_down)
            {
                servo_dropper.setPosition(ArmConstants.servo_dropper_drop_pos);
                servo_arm.setPosition(ArmConstants.servo_arm_drop_pos_wait);
            }
            else if(gamepad1.dpad_left)
            {
                servo_arm.setPosition(ArmConstants.servo_arm_drop_pos_drop);
            }

            if(gamepad1.y)
            {
                servo_arm.setPosition(servo_arm.getPosition() + 0.001);
            }
            else if(gamepad1.b)
            {
                servo_arm.setPosition(servo_arm.getPosition() - 0.001);
            }

            if(gamepad1.x)
            {
                servo_dropper.setPosition(servo_dropper.getPosition() + 0.001);
//                servo_dropper.setPosition(ArmConstants.servo_dropper_drop_pos);
//                servo_arm.setPosition(ArmConstants.servo_arm_drop_pos);
            }
            else if(gamepad1.a)
            {
                servo_dropper.setPosition(servo_dropper.getPosition() - 0.001);
//                servo_dropper.setPosition(ArmConstants.servo_dropper_pickup_pos);
//                servo_arm.setPosition(ArmConstants.servo_arm_pickup_pos);
            }

            telemetry.addData("servo arm position",servo_arm.getPosition());
            telemetry.addData("servo dropper position",servo_dropper.getPosition());
            telemetry.update();
        }
    }
}
