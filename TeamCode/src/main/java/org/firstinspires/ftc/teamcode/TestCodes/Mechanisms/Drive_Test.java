package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import dalvik.system.DelegateLastClassLoader;

//@Disabled
@TeleOp(name="Drive Test", group="Test")
public class Drive_Test extends LinearOpMode{

    public enum Alliance{
        Blue,
        Red
    }
    int TURRETCENTER = 0;
    int TURRETPICKPOS = 0;
    int extension_flag = 0;
    ArmModes mode = ArmModes.INSIDE;
    Alliance team = Alliance.Blue;
    DcMotorEx extension_motor, turnTable, intake;
    Servo servo_dropper, servo_arm, left_intake, right_intake;
    RevColorSensorV3 left_intake_sensor,  right_intake_sensor;
    int MAXEXTENSION = -2500;
    int intakeFlag = 0, transferFlag = 0, droppingFlag=0, servoExtensionFlag = 0;

    public ElapsedTime seconds = new ElapsedTime();



    @Override
    public void runOpMode() {
        switch(team){
            case Blue:
                TURRETCENTER = 243;
                TURRETPICKPOS = 150;
                break;
            case Red:
                TURRETCENTER = -243;
                TURRETPICKPOS = -150;
            default:
                TURRETCENTER = 0;
                TURRETPICKPOS = 0;
        }

        extension_motor = hardwareMap.get(DcMotorEx.class, "extension");
        servo_arm = hardwareMap.get(Servo.class, "Arm");
        servo_dropper = hardwareMap.get(Servo.class, "Dropper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turnTable = hardwareMap.get(DcMotorEx.class, "turret");
        left_intake = hardwareMap.get(Servo.class, "leftIntake");
        right_intake = hardwareMap.get(Servo.class, "rightIntake");

            left_intake_sensor = hardwareMap.get(RevColorSensorV3.class, "leftSensor");
            right_intake_sensor = hardwareMap.get(RevColorSensorV3.class, "rightSensor");



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            extension_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servo_dropper.setPosition(0.3);
        servo_arm.setPosition(0.02);
        left_intake.setPosition(0);
        right_intake.setPosition(1);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        extension_motor.setTargetPosition(0);
        extension_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor.setVelocity(500);
        rotateTo(TURRETPICKPOS);
        seconds.reset();




        while (!isStopRequested()) {

            telemetry.addData("Sensor Val: ", left_intake_sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Sensor Val: ", right_intake_sensor.getDistance(DistanceUnit.MM));

            //##########################################################################
            //############################ INTAKE ######################################
            //##########################################################################
            if(gamepad1.start) {
                intakeFlag = 1;
                transferFlag = 0;
                right_intake.setPosition(1);
            }
            if(gamepad1.back){
                intakeFlag = 0;
            }
            //##########################################################################
            //############################ EXTENSION ###################################
            //##########################################################################
            if(gamepad1.y){
                extension_flag = 1;
            }
            if(gamepad1.a){
                extension_flag = 2;
            }
            if(gamepad1.b){
                extension_flag = 0;
            }

            if(extension_flag == 0){
                extension_motor.setTargetPosition(0);
                extension_motor.setVelocity(0);
            }
            else if(extension_flag == 1) {
                servo_arm.setPosition(0.05);
                extension_motor.setTargetPosition(MAXEXTENSION);
                extension_motor.setVelocity(900);
                telemetry.addData("Status", "Opening");
            }
            else if(extension_flag == 2){
                extension_motor.setTargetPosition(0);
                extension_motor.setVelocity(900);

                telemetry.addData("Status", "Closing");
            }

            //##########################################################################
            //#################### EXTENSION SERVOS ####################################
            //##########################################################################


            if(gamepad1.right_bumper){
                droppingFlag = 1;
            }
            else if(gamepad1.left_bumper){
                droppingFlag = 0;
            }

            if(gamepad1.dpad_down){
                servoExtensionFlag = 0;
                droppingFlag = 0;
            }
            if(gamepad1.dpad_up){
                servoExtensionFlag = 1;
                droppingFlag = 1;
            }

            if(servoExtensionFlag == 1) {
                servo_dropper.setPosition(droppingFlag);
                servo_arm.setPosition(1);
            }
            else if(servoExtensionFlag == 0){
                servo_dropper.setPosition(0.3);
                servo_arm.setPosition(0.02);
            }


            //##########################################################################
            //############################ INTAKE SERVOS ###############################
            //##########################################################################

            if(intakeFlag == 1){
                if(right_intake_sensor.getDistance(DistanceUnit.MM) < 20){
                    setIntakeTo(0);
                    telemetry.addData("Object ", "Detected");
                    transferFlag = 1;
                    intakeFlag = 0;
                    right_intake.setPosition(0);
                    seconds.reset();
                }
                else{
                    setIntakeTo(-1);
                    telemetry.addData("Object ", "Not Detected");
                }
            }
            else if(intakeFlag == 0){
                setIntakeTo(0);
            }

            if((transferFlag == 1) && (seconds.milliseconds() > 1000)){
                if((right_intake.getPosition() == 0) && (right_intake_sensor.getDistance(DistanceUnit.MM) < 20)){
                    setIntakeTo(-1);
                }
                else{
                    setIntakeTo(0);
                }
            }

            //##########################################################################
            //############################ TURRET ######################################
            //##########################################################################
            int pos = turnTable.getCurrentPosition();
            if(gamepad1.right_trigger > 0.5){
                pos = pos + 20;
                rotateTo(pos);
            }
            else if(gamepad1.left_trigger > 0.5){
                pos = pos - 20;
                rotateTo(pos);
            }



            //##########################################################################
            //############################## DRIVE #####################################
            //##########################################################################
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("extension pos: ", extension_motor.getCurrentPosition());
            telemetry.addData("turret pos: ", turnTable.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void rotateTo(int pos){
        turnTable.setTargetPosition(pos);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setVelocity(500);
    }

    public void extendTo(int pos){
        extension_motor.setTargetPosition(0);
        extension_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor.setVelocity(500);
    }

    public void setIntakeTo(int power){
        intake.setPower(power);
    }

    public void armPosition(ArmModes modes){
        if(modes == ArmModes.INSIDE){
            servo_arm.setPosition(0);
            servo_dropper.setPosition(0);
        }
        else if(modes ==  ArmModes.EXTENDED){
            servo_arm.setPosition(1);
            servo_dropper.setPosition(1);
        }
        else if(modes ==  ArmModes.DROP){
            if(extension_motor.getCurrentPosition() > MAXEXTENSION/2) {
                servo_arm.setPosition(1);
                servo_dropper.setPosition(0);
            }
        }
    }

    public enum ArmModes{
        INSIDE,
        EXTENDED,
        DROP
    }
}
