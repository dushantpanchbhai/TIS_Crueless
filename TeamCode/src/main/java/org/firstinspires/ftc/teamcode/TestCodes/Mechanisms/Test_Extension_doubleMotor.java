package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Extension Test with 2 motors", group="Linear OpMode")
@Disabled
public class Test_Extension_doubleMotor extends LinearOpMode{

    DcMotorEx motor1, turnTable, intake;
    Servo servo_dropper, servo_arm, left_intake, right_intake;
    int MOTOR1MAX = 2500;

    int flag = 0, pos=0;
    float stick_val, stick_val_prev=0;



    @Override
    public void runOpMode(){
    motor1 = hardwareMap.get(DcMotorEx.class, "motor");
    servo_arm = hardwareMap.get(Servo.class, "Arm");
    servo_dropper = hardwareMap.get(Servo.class, "Dropper");
    intake = hardwareMap.get(DcMotorEx.class, "intake");
    turnTable = hardwareMap.get(DcMotorEx.class, "turret");
    left_intake = hardwareMap.get(Servo.class, "leftIntake");
    right_intake = hardwareMap.get(Servo.class, "rightIntake");

    motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    servo_arm.setPosition(0);
    servo_dropper.setPosition(0);
    left_intake.setPosition(0);
    right_intake.setPosition(0);

    waitForStart();

    motor1.setTargetPosition(0);
    turnTable.setTargetPosition(0);

    motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor1.setVelocity(500);
    turnTable.setVelocity(500);

    while(opModeIsActive()){

        stick_val = gamepad1.left_stick_x;

        if(stick_val > stick_val_prev) {
            pos++;
        }
        else if(stick_val < stick_val_prev){
            pos--;
        }

        if(gamepad1.right_bumper){
            flag = 1;
        }
        if(gamepad1.left_bumper){
            flag = 2;
        }

        if(gamepad1.b){
            flag = 0;
        }

        if(gamepad1.dpad_left){
            servo_arm.setPosition(1);
            sleep(10);
            servo_dropper.setPosition(1);
        }

        if(gamepad1.start){
            servo_dropper.setPosition(1);
        }

        if(gamepad1.back){
            servo_dropper.setPosition(0);
        }

        if(gamepad1.dpad_right){
            servo_dropper.setPosition(0);
            sleep(10);
            servo_arm.setPosition(0);
        }

        if(gamepad1.right_stick_button){
            intake.setPower(-1);
        }
        else if(gamepad1.left_stick_button){
            intake.setPower(0);
        }

        if(flag == 0){
            motor1.setTargetPosition(0);
            motor1.setVelocity(0);
        }
        else if(flag == 1){
            motor1.setTargetPosition(MOTOR1MAX);
            motor1.setVelocity(900);
            telemetry.addData("Status", "Opening");
        }
        else if(flag == 2){
            motor1.setTargetPosition(0);
            motor1.setVelocity(900);
            telemetry.addData("Status", "Closing");
        }

        if(gamepad1.dpad_down){
            left_intake.setPosition(0);
            right_intake.setPosition(0);
        }
        if(gamepad1.dpad_up){
            left_intake.setPosition(1);
            right_intake.setPosition(1);
        }


        turnTable.setTargetPosition(pos);
        turnTable.setVelocity(400);
        telemetry.addData("Motor 1: ", motor1.getCurrentPosition());
        telemetry.update();
    }
    }
}
