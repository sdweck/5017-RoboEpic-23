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
@Autonomous(name = "AutoBlueLeft", group = "drive")
public class AutoBlueLeft extends LinearOpMode{

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
            LiftUpForTime(-1, 4.0);
            LIFT.setPower(0);
            Trajectory StrafetoSignalCone = drive.trajectoryBuilder(new Pose2d())
                    .forward(17)
                    .build();
            drive.followTrajectory(StrafetoSignalCone);
            Trajectory StrafetoSenseSignalCone = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .strafeRight(7)
                    .build();
            drive.followTrajectory(StrafetoSenseSignalCone);

            while (COLORSENSOR.red() == 0 && opModeIsActive()) {
                // crab to the righct
                telemetry.addData("Red", COLORSENSOR.red());
                telemetry.addData("Green", COLORSENSOR.green());
                telemetry.addData("Blue", COLORSENSOR.blue());
                telemetry.update();
            }
            double redVal = COLORSENSOR.red();
            double greenVal = COLORSENSOR.green();
            double blueVal = COLORSENSOR.blue();

            Trajectory StrafetoRecenterFromSignalCone = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .strafeLeft(7)
                    .build();
            drive.followTrajectory(StrafetoRecenterFromSignalCone);
            //Forward to Medium Junction
            Trajectory ForwardtoMedJunction = drive.trajectoryBuilder(StrafetoSenseSignalCone.end())
                    .forward(22)
                    .build();
            drive.followTrajectory(ForwardtoMedJunction);
            Trajectory StrafeRightTowardJunction = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .strafeRight(7)
                    .build();
            drive.followTrajectory(StrafeRightTowardJunction);
            LiftUpForTime(.7, .5);
            INTAKE.setPosition(.25);
            Trajectory AligntoPark = drive.trajectoryBuilder(StrafetoSignalCone.end())
                    //TEST THE STRAFING VALUE//
                    .back(13)
                    .build();
            drive.followTrajectory(AligntoPark);
            //Lift up
           /* Trajectory ForwardtoAlignwithStack = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                    .forward(13)
                    .build();
            drive.followTrajectory(ForwardtoAlignwithStack);*/
            /*drive.turn(Math.toRadians(90));
            Trajectory BackwardstoStackJunction = drive.trajectoryBuilder(ForwardtoAlignwithStack.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                    .back(29)
                    .build();
            drive.followTrajectory(BackwardstoStackJunction);
            ARM.setPosition(.855);
            LiftUpForTime(1, .5);*/


         /*ajectory StrafeRightoScoreMedJunction = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                    .strafeRight(3)
                    .build();
            drive.followTrajectory(StrafeRightoScoreMedJunction);
            //CONE DROP
            Trajectory StrafeLefttoRecenter = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                    .strafeLeft(3)
                    .build();
            drive.followTrajectory(StrafeLefttoRecenter);


            //Lower Lift
            //Pick Up Stack
            //Lift Lift

            Trajectory StrafeRightoAlignHighJunction = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                    .strafeRight(3)
                    .build();
            drive.followTrajectory(StrafeRightoAlignHighJunction);*/
            //swing arm back right
            //lift lidt
            //drop lidt
            //drop cone
            //lift up
            //park in signal zone
            if (redVal > greenVal && redVal > blueVal) {
                Trajectory Red = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                        .strafeLeft(37)
                        .build();
                drive.followTrajectory(Red);
                telemetry.addData("red", "signal");
                telemetry.update();
                sleep(3000);


            }
            // if blue go to zone 2 (already there no if statement)
            else if (blueVal > redVal && blueVal > greenVal) {
                /*Trajectory Blue = drive.trajectoryBuilder(ForwardtoMedJunction.end())
                        .back(12)
                        .build();*/
               // drive.followTrajectory(Blue);
                telemetry.addData("blue", "signal");
                telemetry.update();
                sleep(3000);
            } else if (greenVal > redVal && greenVal > blueVal) {
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

    private void CrabForDistance(double power, double revolutions) {
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