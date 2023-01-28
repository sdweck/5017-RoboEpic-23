package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import java.awt.

@Config
@Autonomous(name = "TESTING2", group = "drive")
public class TESTING extends LinearOpMode{

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
           /* Trajectory Forward = drive.trajectoryBuilder(new Pose2d())
                    .forward(40)
                    .build();
            drive.followTrajectory(Forward);*/
           Trajectory StrafeL = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(40)
                    .build();
            drive.followTrajectory(StrafeL);
           /* Trajectory StrafeR = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(20)
                    .build();
            drive.followTrajectory(StrafeR);
            Trajectory Back = drive.trajectoryBuilder(new Pose2d())
                    .back(24)
                    .build();
            drive.followTrajectory(Back);*/


            //Testing Small Junction
           /* ARM.setPosition(.155);
            LiftUpForTime(-1, 1.5);
            LiftUpForTime(1,.99);
            //Cone expand to grab cone = .8, Cone drop cone =.25!!
            INTAKE.setPosition(.8);
            sleep(300);
            LiftUpForTime(-1, 2);

            Trajectory toStack1 = drive.trajectoryBuilder(new Pose2d())
                    .back(35)
                    .build();
            drive.followTrajectory(toStack1);
            Trajectory toMedJunc = drive.trajectoryBuilder(toStack1.end())
                    .strafeRight(10)
                    .build();
            drive.followTrajectory(toMedJunc);

            ARM.setPosition(.5);
            sleep(500);
            LiftUpForTime(1, 0.25);
            sleep(700);
            INTAKE.setPosition(.25);
            sleep(500);
            LiftUpForTime(-1, 0.25);


            Trajectory Back = drive.trajectoryBuilder(toStack1.end())
                    .back(5)
                    .build();
            drive.followTrajectory(Back);
            Trajectory toMedJunc2 = drive.trajectoryBuilder(toStack1.end())
                    .strafeLeft(10)
                    .build();
            drive.followTrajectory(toMedJunc2);
            Trajectory toStack2 = drive.trajectoryBuilder(toStack1.end())
                    .forward(43)
                    .build();
            drive.followTrajectory(toStack2);
            ARM.setPosition(.155);
            LiftUpForTime(1,1.25);
            sleep(800);
            INTAKE.setPosition(.8);
            sleep(800);
            LiftUpForTime(-1,3.40);
            Trajectory toTallJunc1 = drive.trajectoryBuilder(toStack2.end())
                    .back(26)
                    .build();
            drive.followTrajectory(toTallJunc1);
            Trajectory toTallJunc2 = drive.trajectoryBuilder(toStack2.end())
                    .strafeLeft(17)
                    .build();
            drive.followTrajectory(toTallJunc2);
            Trajectory For = drive.trajectoryBuilder(toTallJunc2.end())
                            .back(3)
                                    .build();
            drive.followTrajectory(For);
            ARM.setPosition(.855);
            sleep(1000);
            INTAKE.setPosition(.25);
            Trajectory toTallJunc3 = drive.trajectoryBuilder(toStack2.end())
                    .strafeRight(17)
                    .build();
            drive.followTrajectory(toTallJunc3);*/


            //Testing for Cone
            /*Trajectory Forward1 = drive.trajectoryBuilder(new Pose2d())
                    .forward()
                     .build();
            drive.followTrajectory(Forward1);
            Trajectory Crab1 = drive.trajectoryBuilder(Forward1.end())
                    //could be left as well, not sure
                    .strafeRight()
                    .build();
            drive.followTrajectory(Crab1);

            //Testing Medium
            LiftUpForTime(-1, 1.8);
            ARM.setPosition(.5);
            LiftUpForTime(1, 1.25);
            sleep(300);
            //Cone expand to grab cone = .8, Cone drop cone =.25!!
            INTAKE.setPosition(.8);
            LiftUpForTime(-1,1.8);
            Trajectory toStack2 = drive.trajectoryBuilder(new Pose2d())
                    .back(40)
                    .build();
            drive.followTrajectory(toStack2);
            LiftUpForTime(1, 1.15);
            INTAKE.setPosition(.25);
            LiftUpForTime(-1,1);


            //Testing Large
            LiftUpForTime(-1, 1.8);
            ARM.setPosition(.5);
            LiftUpForTime(1, 1.25);
            sleep(300);
            //Cone expand to grab cone = .8, Cone drop cone =.25!!
            INTAKE.setPosition(.8);
            LiftUpForTime(-1,1.8);
            Trajectory toStack2 = drive.trajectoryBuilder(new Pose2d())
                    .back(40)
                    .build();
            drive.followTrajectory(toStack2);
            LiftUpForTime(1, 1.15);
            INTAKE.setPosition(.25);
            LiftUpForTime(-1,1);

            //The arm for the Low and Medium has to swing (if facing the same way the back of the robot is facing) to the right. Big = left

            //1. Test the heights of the junctions with amount of intake time/lift up AND down
            //2.Test positioning of arm




*/



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