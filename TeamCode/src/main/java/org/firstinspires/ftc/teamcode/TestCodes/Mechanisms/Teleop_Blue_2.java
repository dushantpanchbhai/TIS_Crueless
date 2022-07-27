package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name="Teleop_Blue 2", group="Final OpModes")
public class Teleop_Blue_2 extends LinearOpMode{


    public ElapsedTime seconds = new ElapsedTime();
    //####################### TURRET POSITIONS #########################
    int TURRETCENTER = 243;
    int TURRETPICKPOS = 130;

    //##################################################################
    //########################## FLAGS #################################
    int BEGININTAKE = 1;
    int STOPINTAKE = 0;
    int BEGINTRANSFER = 1;
    int STOPTRANSFER = 0;
    int intakeFlag = 0, manualIntakeFlag = 0;
    int transferFlag = 0;

    //##################################################################
    //####################### EXTENSION POSITIONS ######################
    int pos = 0;

    //##################################################################
    //############################# HARDWARE ###########################
    DcMotorEx extension_motor1, extension_motor2, turnTable, intake;
    Servo servo_dropper, servo_arm, left_intake, right_intake;
    RevColorSensorV3 left_intake_sensor,  right_intake_sensor;




    public void runOpMode() {
        //##################################################################
        //####################### HARDWARE MAP #############################
        extension_motor1 = hardwareMap.get(DcMotorEx.class, "extension1");
        extension_motor2 = hardwareMap.get(DcMotorEx.class, "extension2");
        servo_arm = hardwareMap.get(Servo.class, "Arm");
        servo_dropper = hardwareMap.get(Servo.class, "Dropper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turnTable = hardwareMap.get(DcMotorEx.class, "turret");
        left_intake = hardwareMap.get(Servo.class, "leftIntake");
        right_intake = hardwareMap.get(Servo.class, "rightIntake");
        left_intake_sensor = hardwareMap.get(RevColorSensorV3.class, "leftSensor");
        right_intake_sensor = hardwareMap.get(RevColorSensorV3.class, "rightSensor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        extension_motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension_motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extension_motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        ################# INITIAL POSITIONS ###############################

        left_intake.setPosition(0);
        right_intake.setPosition(1);

        TrajectorySequence DropBall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    servo_dropper.setPosition(ArmConstants.servo_dropper_mid);
                    servo_arm.setPosition(ArmConstants.servo_arm_mid);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    pos = ArmConstants.extension_max_pos;
                    extendTo(pos);
                }).waitSeconds(1.5)
                .addTemporalMarker(()->{
                    servo_dropper.setPosition(ArmConstants.servo_dropper_drop_pos);
                    servo_arm.setPosition(ArmConstants.servo_arm_drop_pos_wait);
                })
                .build();

        TrajectorySequence BackToInitialPos = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(()->{
                    servo_dropper.setPosition(ArmConstants.servo_dropper_mid);
                    servo_arm.setPosition(ArmConstants.servo_arm_mid);
                }).waitSeconds(0.2)
                .addTemporalMarker(()->{
                    pos = ArmConstants.extension_low_pos;
                    extendTo(pos);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    servo_dropper.setPosition(ArmConstants.servo_dropper_pickup_pos);
                    servo_arm.setPosition(ArmConstants.servo_arm_pickup_pos);
                })
                .build();

        waitForStart();

        servo_dropper.setPosition(ArmConstants.servo_dropper_pickup_pos);
        servo_arm.setPosition(ArmConstants.servo_arm_pickup_pos);

        extension_motor1.setTargetPosition(0);
        extension_motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor1.setVelocity(500);
        extension_motor2.setTargetPosition(0);
        extension_motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor2.setVelocity(500);
        rotateTo(TURRETPICKPOS);
        seconds.reset();

        while (opModeIsActive() || !isStopRequested()) {

//          ################# TURRET LOGIC ####################
            if(gamepad1.start){
                rotateTo(TURRETCENTER);
            }
            else if(gamepad1.back){
                rotateTo(TURRETPICKPOS);
            }

//          ################# INTAKE LOGIC ####################
            if(gamepad1.dpad_up){
                intakeFlag = BEGININTAKE;                //Start the Auto Intake Process
                manualIntakeFlag = STOPINTAKE;           //Stop the Manual Intake Process
                transferFlag = STOPTRANSFER;             //Stop transfers from taking place
            }
            if(gamepad1.dpad_down){
                intakeFlag = STOPINTAKE;
                manualIntakeFlag = STOPINTAKE;
                transferFlag = STOPTRANSFER;
            }

            if((intakeFlag == BEGININTAKE) && !(getProximityReading(right_intake_sensor, 20)) && (manualIntakeFlag == STOPINTAKE)){
                right_intake.setPosition(1);
                setIntakeTo(-1);                   //If Auto Intake is Enabled and sensor readings are over 20, start Intake
            }
            else if((intakeFlag == BEGININTAKE) && (getProximityReading(right_intake_sensor, 20)) && (manualIntakeFlag == STOPINTAKE)){
                setIntakeTo(0);                    //Turn off the Intake since Object has been detected
                transferFlag = BEGINTRANSFER;                  //Ready for Transfer
                intakeFlag = STOPINTAKE;
                right_intake.setPosition(0);       //Rotate the servo to make the intake ready for transfer
                seconds.reset();                   //Reset the timer for transfer counts
            }
            else if((intakeFlag == STOPINTAKE) && (manualIntakeFlag == STOPINTAKE)){
                setIntakeTo(0);                    //Turn the Intake off
            }
            else if((intakeFlag == STOPINTAKE) && (manualIntakeFlag == BEGININTAKE)){
                setIntakeTo(-1);                   //Turn the Intake on
            }

            if((transferFlag == BEGINTRANSFER) && (seconds.milliseconds() > 1000)){
                if(getProximityReading(right_intake_sensor, 20)){
                    setIntakeTo(-1);
                }
            }

            if(transferFlag == BEGINTRANSFER){
                if(!getProximityReading(right_intake_sensor, 20)){
                    setIntakeTo(0);
                }
            }

            telemetry.addData("intakeFlag = ", intakeFlag);
            telemetry.addData("transferFlag = ", transferFlag);
            telemetry.addData("manualIntakeFlag = ", manualIntakeFlag);
            telemetry.addData("Encoder 1 count: ", extension_motor1.getCurrentPosition());
            telemetry.addData("Encoder 2 count: ", extension_motor2.getCurrentPosition());
            telemetry.addData("Motor 1 current: ", extension_motor1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor 2 current: ", extension_motor2.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

//          ##################### ARM EXTENSION + BALL DROP ###########################
            if(gamepad1.y)
            {
                drive.followTrajectorySequence(DropBall);
            }
            else if(gamepad1.b)
            {
                drive.followTrajectorySequence(BackToInitialPos);
            }

            if(gamepad1.a)
            {
                servo_arm.setPosition(ArmConstants.servo_arm_drop_pos_drop);
            }

//         #################### TURN TABLE MOVEMENT ##########################
            if(gamepad1.dpad_left)
            {
                turnTable.setTargetPosition(turnTable.getCurrentPosition() - ArmConstants.turn_table_inc);
            }
            else if(gamepad1.dpad_right)
            {
                turnTable.setTargetPosition(turnTable.getCurrentPosition() + ArmConstants.turn_table_inc);
            }

            if(gamepad1.x)
            {
                setIntakeTo(-1);
            }
            else
            {
                setIntakeTo(0);
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();



            Pose2d poseEstimate = drive.getPoseEstimate();

        }
    }


    public void rotateTo(int pos){
        turnTable.setTargetPosition(pos);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setVelocity(500);
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

    public void setIntakeTo(int power){
        intake.setPower(power);
    }

    public boolean getProximityReading(RevColorSensorV3 sensor, int dist){
        if(sensor.getDistance(DistanceUnit.MM) < dist){
            return true;
        }
        else{
            return false;
        }
    }
    int i_error, prev_error;
    float KP_catch = 1, KI_catch = 0, KD_catch = 0;
    public int catchUpPID(int current, int target){
        int error = target - current;
        int p_error = error;
        i_error = error + i_error;
        int d_error = error - prev_error;
        return (int) ((KP_catch * p_error) + (KI_catch * i_error) + (KD_catch * d_error));
    }


}
