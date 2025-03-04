package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import java.awt.

@Config
@Autonomous(name = "AutoRedRight", group = "drive")
public class AutoRedRight extends LinearOpMode{

    // Instance variables corresponding to our various motors/servos.
    private DcMotor LEFTBACK; //2:0
    private DcMotor RIGHTBACK; //2:1
    private DcMotor RIGHTFRONT; //1:0
    private DcMotor LEFTFRONT; //1:1
    private Servo INTAKE;
    private Servo ARM;
    private ColorSensor COLORSENSOR;
    private DcMotor LIFT;
    private ElapsedTime runtime = new ElapsedTime();

    final double encRotation = 537.6;

    public void runOpMode() {
        // These strings need to match the config on the robot.
        LEFTBACK = hardwareMap.dcMotor.get("LEFTBACK");
        RIGHTBACK = hardwareMap.dcMotor.get("RIGHTBACK");
        RIGHTFRONT = hardwareMap.dcMotor.get("RIGHTFRONT");
        LEFTFRONT = hardwareMap.dcMotor.get("LEFTFRONT");
        COLORSENSOR = hardwareMap.get(ColorSensor.class, "COLORSENSOR");
        LIFT = hardwareMap.dcMotor.get("LIFT");
        INTAKE = hardwareMap.servo.get("INTAKE");
        ARM = hardwareMap.servo.get("ARM");

        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new
                SampleMecanumDrive(hardwareMap);
        // Wait for the game to start (driver presses PLAY)y77
        waitForStart();

        if (opModeIsActive()) {
            // raise the lift
            LiftUpForTime(-0.7, 3);
            LIFT.setPower(0);
            Trajectory StrafetoSignalCone = drive.trajectoryBuilder(new Pose2d())
                    .forward(19)
                    .build();
            drive.followTrajectory(StrafetoSignalCone);
            Trajectory StrafetoSenseSignalCone = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .strafeRight(7)
                    .build();
            drive.followTrajectory(StrafetoSenseSignalCone);
            while (COLORSENSOR.red() == 0 && opModeIsActive()){
                // crab to the right
                telemetry.addData("Red", COLORSENSOR.red());
                telemetry.addData("Green", COLORSENSOR.green());
                telemetry.addData("Blue", COLORSENSOR.blue());
                telemetry.update();
            }

            double redVal = COLORSENSOR.red();
            double greenVal = COLORSENSOR.green();
            double blueVal = COLORSENSOR.blue();

            Trajectory StrafeAwayfromSignalCone = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .strafeLeft(7)
                    .build();
            drive.followTrajectory(StrafeAwayfromSignalCone);

            //Forward to Medium Junction
            Trajectory ForwardtoMedJunction = drive.trajectoryBuilder(StrafetoSenseSignalCone.end())
                    .forward(23)
                    .build();
            drive.followTrajectory(ForwardtoMedJunction);
            // strafe right toward junction
            Trajectory StrafeRightTowardJunction = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .strafeRight(5)
                    .build();
            drive.followTrajectory(StrafeRightTowardJunction);

            // Cone Drop
            LiftUpForTime(0.7, 0.5);
            INTAKE.setPosition(0.25);
            sleep(2000);

            // left to recenter
            Trajectory StrafeLefttoRecenter = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                    .strafeLeft(7)
                    .build();
            drive.followTrajectory(StrafeLefttoRecenter);

            // backwords to get to signal row 1
            Trajectory BackToGetToSignalRow = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                    .back(14)
                    .build();
            drive.followTrajectory(BackToGetToSignalRow);

            // swing arm back
            ARM.setPosition(0.83);
            sleep(2000);

////            Trajectory ForwardtoAlignwithStack = drive.trajectoryBuilder(ForwardtoMedJunction.end())
////                    .forward(15)
//                    .build();
//            drive.followTrajectory(ForwardtoAlignwithStack);
//            Trajectory StrafetoConeStack = drive.trajectoryBuilder(ForwardtoMedJunction.end())
//                    .strafeRight(23)
//                    .build();
//            drive.followTrajectory(StrafetoConeStack);
//            Trajectory StrafeRightoAlignHighJunction = drive.trajectoryBuilder(ForwardtoMedJunction.end())
//                    .strafeRight(3)
//                    .build();
//            drive.followTrajectory(StrafeRightoAlignHighJunction);
//            //Lower Lift
//            // TEST THE TIME VALUE //
//            LiftUpForTime(-0.7, 0.5);
//            //Pick Up Cone
//            //Raise Lift
//            // TEST THE TIME VALUE //
//            LiftUpForTime(0.7, 3.5);
//            //Swimg Arm Forward
//            Trajectory StrafeLefttoHighJunction = drive.trajectoryBuilder(ForwardtoMedJunction.end())
//                    .strafeLeft(36)
//                    .build();
//            drive.followTrajectory(StrafeLefttoHighJunction);
//            Trajectory ForwardtoScoreHighJunction = drive.trajectoryBuilder(ForwardtoMedJunction.end())
//                    .forward(5)
//                    .build();
//            drive.followTrajectory(ForwardtoScoreHighJunction);
//            Trajectory BacktoPark = drive.trajectoryBuilder(ForwardtoMedJunction.end())
//                    .back(2)
//                    .build();
//            drive.followTrajectory(BacktoPark);
//            //Lower Lift
//            LiftUpForTime(-0.7,1);
//            //Drop Cone
//
            // if red go to signal zone 1
            if (redVal > greenVal && redVal > blueVal) {
                Trajectory Red = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                        .strafeLeft(35)
                        .build();
                drive.followTrajectory(Red);
                telemetry.addData("red", "signal");
                telemetry.update();
                sleep(3000);


            }

            // if blue go to signal zone 2
            else if (blueVal > greenVal && blueVal > redVal) {
//                Trajectory Red = drive.trajectoryBuilder(BacktoPark.end())
//                        .strafeRight(15)
//                        .build();
//                drive.followTrajectory(Red);
                telemetry.addData("blue", "signal");
                telemetry.update();
                sleep(3000);


            }
            // if green go to signal zone 3
            else if (greenVal > redVal && greenVal > blueVal) {
                Trajectory Green = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                        .strafeRight(35)
                        .build();
                drive.followTrajectory(Green);
                telemetry.addData("green", "signal");
                telemetry.update();
                sleep(3000);
            }

            else{
                telemetry.addData("no color", "sensed");
                telemetry.addData("red: ", redVal);
                telemetry.addData("green: ", greenVal);
                telemetry.addData("blue: ", blueVal);
                telemetry.update();
                sleep(3000);
            }


        }
    }

    private void stopEverything() {
        LEFTFRONT.setPower(0);
        RIGHTFRONT.setPower(0);
        LEFTBACK.setPower(0);
        RIGHTBACK.setPower(0);
    }

    private void LiftUpForTime(double power, double time){
        runtime.reset();
        while (runtime.seconds() <= time) {
            telemetry.addData("lift", "function");
            telemetry.update();
            LIFT.setPower(power);
        }
        LIFT.setPower(0);
    }

    private void ForwardForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setTargetPosition(denc);
        LEFTBACK.setTargetPosition(denc);
        RIGHTBACK.setTargetPosition(denc);
        LEFTFRONT.setTargetPosition(denc);

        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);

        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }

        stopEverything();
    }
    //This is not using roadrunner! - DO NOT USE, not deleting as I think it affects the CrabforDistance function
    private void TurnForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.REVERSE);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setTargetPosition(denc);
        LEFTBACK.setTargetPosition(denc);
        RIGHTBACK.setTargetPosition(denc);
        LEFTFRONT.setTargetPosition(denc);

        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "running");
        telemetry.update();

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);
        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }
        stopEverything();
    }

    private void CrabForDistance (double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        RIGHTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        LEFTFRONT.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHTBACK.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFTBACK.setDirection(DcMotorSimple.Direction.REVERSE);

        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RIGHTFRONT.setTargetPosition(denc);
        LEFTBACK.setTargetPosition(denc);
        RIGHTBACK.setTargetPosition(denc);
        LEFTFRONT.setTargetPosition(denc);

        LEFTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LEFTFRONT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTBACK.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "running");
        telemetry.update();

        RIGHTFRONT.setPower(power);
        LEFTFRONT.setPower(power);
        RIGHTBACK.setPower(power);
        LEFTBACK.setPower(power);

        while (opModeIsActive() && LEFTBACK.isBusy() && LEFTFRONT.isBusy() && RIGHTBACK.isBusy() && RIGHTFRONT.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            idle();
        }

        stopEverything();
    }

    private void resetEncoders() {
        LEFTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTFRONT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}