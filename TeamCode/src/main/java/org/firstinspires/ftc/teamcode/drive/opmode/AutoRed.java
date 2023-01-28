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
@Autonomous(name = "AutoRed", group = "drive")
public class AutoRed extends LinearOpMode{

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
            // START WITH PRELOAD LIFTED
            LiftUpForTime(-1, 1.8);
            // GO FORWARD TO READ SPECIAL CONE
            Trajectory goForward1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(17 )
                    .build();
            drive.followTrajectory(goForward1);
            Trajectory D = drive.trajectoryBuilder(goForward1.end())
                    .strafeRight(3)
                    .build();
            drive.followTrajectory(D);
            LiftUpForTime(1, 0.25);
            INTAKE.setPosition(.25);
            Trajectory goForward2 = drive.trajectoryBuilder(goForward1.end())
                    .forward(5)
                    .build();
            drive.followTrajectory(goForward2);
            Trajectory E = drive.trajectoryBuilder(goForward2.end())
                    .strafeRight(10)
                    .build();
            drive.followTrajectory(E);

            Trajectory Strait = drive.trajectoryBuilder(E.end())
                    .forward(10)
                    .build();
            drive.followTrajectory(Strait);

            /*Trajectory CrabABit = drive.trajectoryBuilder(goForward1.end())
                    .strafeRight(2)
                    .build();
            drive.followTrajectory(CrabABit);*/
           /* Trajectory goForwardATad = drive.trajectoryBuilder(CrabABit.end())
                    .forward(7)
                    .build();
            drive.followTrajectory(goForwardATad);*/

            // READ AND STORE COLOR OF SPECIAL CONE
            while (COLORSENSOR.red() == 0 && opModeIsActive()){
                // crab to the righct
                telemetry.addData("Red", COLORSENSOR.red());
                telemetry.addData("Green", COLORSENSOR.green());
                telemetry.addData("Blue", COLORSENSOR.blue());
                telemetry.update();
            }

            double redVal = COLORSENSOR.red();
            double greenVal = COLORSENSOR.green();
            double blueVal = COLORSENSOR.blue();

            /*Trajectory goPosition = drive.trajectoryBuilder(goForward1.end())
                    .forward(20)
                    .build();
            drive.followTrajectory(goPosition);


            Trajectory Crab1 = drive.trajectoryBuilder(goPosition.end())
                    .strafeRight(5)
                    .build();
            drive.followTrajectory(Crab1);*/
            //
            // PRELOAD
            /*Trajectory goBack = drive.trajectoryBuilder(Crab1.end())
                    .back(5)
                    .build();
            drive.followTrajectory(goBack);

            INTAKE.setPosition(.25);
            LiftUpForTime(-.10, .1);
            Trajectory goForward2 = drive.trajectoryBuilder(goBack.end())
                    .forward(7)
                    .build();
            drive.followTrajectory(goForward2);
           *//* drive.turn(Math.toRadians(-90));
          Trajectory align = drive.trajectoryBuilder(goForward2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                  .forward(15)
                          .build();
          drive.followTrajectory(align);
          Trajectory side = drive.trajectoryBuilder(align.end())
                  .strafeLeft(38)
                          .build();
            drive.followTrajectory(side);
            Trajectory conez = drive.trajectoryBuilder(align.end())
                    .forward(35)
                    .build();
            drive.followTrajectory(conez);
          //Getting cone from stack
            ARM.setPosition(.155);
            LiftUpForTime(1,.25);
            //Cone expand to grab cone = .8, Cone drop cone =.25!!
            INTAKE.setPosition(.8);
            sleep(300);
            LiftUpForTime(-1, 2);

            Trajectory toStack1 = drive.trajectoryBuilder(conez.end())
                    .back(35)
                    .build();
            drive.followTrajectory(toStack1);
            Trajectory toMedJunc = drive.trajectoryBuilder(toStack1.end())
                    .strafeRight(10)
                    .build();
            drive.followTrajectory(toMedJunc);

            ARM.setPosition(.5);
            sleep(500);
            LiftUpForTime(1, 0.1);
            sleep(700);
            INTAKE.setPosition(.25);
            sleep(500);
            LiftUpForTime(-1, 0.25);

            Trajectory Backz = drive.trajectoryBuilder(toStack1.end())
                    .back(5)
                    .build();
            drive.followTrajectory(Backz);
            Trajectory toMedJunc2 = drive.trajectoryBuilder(Backz.end())
                    .strafeLeft(10)
                    .build();
            drive.followTrajectory(toMedJunc2);
            Trajectory toStack2 = drive.trajectoryBuilder(toMedJunc2.end())
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
            Trajectory toTallJunc2 = drive.trajectoryBuilder(toTallJunc1.end())
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
            Trajectory toTallJunc3 = drive.trajectoryBuilder(For.end())
                    .strafeRight(17)
                    .build();
            drive.followTrajectory(toTallJunc3);*//*
*/
            // if red go to zone 1
            if (redVal > greenVal && redVal > blueVal) {
                Trajectory Red1 = drive.trajectoryBuilder(goForward2.end())
                        .forward(7)
                        .build();
                drive.followTrajectory(Red1);
                Trajectory Red = drive.trajectoryBuilder(Red1.end())
                        .strafeLeft(15)
                        .build();
                drive.followTrajectory(Red);


            }
            // if blue go to zone 2 (already there no if statement)
           /* else if (blueVal  > redVal && blueVal > greenVal) {
                Trajectory Blue = drive.trajectoryBuilder(Crab1.end())
                        .strafeRight(7)
                        .build();
                drive.followTrajectory(Blue);
            }*/


            else if (greenVal > redVal && greenVal > blueVal) {
                Trajectory Red1 = drive.trajectoryBuilder(goForward2.end())
                        .forward(7)
                        .build();
                drive.followTrajectory(Red1);
                Trajectory Green = drive.trajectoryBuilder(Red1.end())
                        .strafeRight(15)
                        .build();
                drive.followTrajectory(Green);


            }
            // ForwardForDistance(.7, -.0001);
            sleep(1000);


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