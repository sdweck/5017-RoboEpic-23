/*
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

public class Auto2 {
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
    public class TrajectoryBuilder extends LinearOpMode {

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
                ARM.setPosition(.855);
                // START WITH PRELOAD LIFTED
                LiftUpForTime(-1, 1.8);
                LIFT.setPower(0);

                // GO FORWARD TO READ SPECIAL CONE
                Trajectory goForward1 = drive.trajectoryBuilder(new Pose2d())
                        .forward(18)
                        .build();
                drive.followTrajectory(goForward1);
                Trajectory CrabABit = drive.trajectoryBuilder(goForward1.end())
                        .strafeRight(5)
                        .build();
                drive.followTrajectory(CrabABit);
                Trajectory goForwardATad = drive.trajectoryBuilder(CrabABit.end())
                        .forward(7)
                        .build();
                drive.followTrajectory(goForwardATad);

                sleep(2000);

                // READ AND STORE COLOR OF SPECIAL CONE
                while (COLORSENSOR.red() == 0){
                    // crab to the righct
                    telemetry.addData("Red", COLORSENSOR.red());
                    telemetry.addData("Green", COLORSENSOR.green());
                    telemetry.addData("Blue", COLORSENSOR.blue());
                    telemetry.update();
                }

                double redVal = COLORSENSOR.red();
                double greenVal = COLORSENSOR.green();
                double blueVal = COLORSENSOR.blue();
                Trajectory goPosition = drive.trajectoryBuilder(goForwardATad.end())
                        .strafeLeft(5)
                        .build();
                drive.followTrajectory(goPosition);
                Trajectory goForwardASmidge = drive.trajectoryBuilder(goForwardATad.end())
                        .forward(12)
                        .build();
                drive.followTrajectory(goForwardASmidge);

                Trajectory Crab1 = drive.trajectoryBuilder(goForward1.end())
                        .strafeLeft(22)
                        .build();
                drive.followTrajectory(Crab1);
                //
                // PRELOAD
                Trajectory goBack = drive.trajectoryBuilder(Crab1.end())
                        .back(5)
                        .build();
                drive.followTrajectory(goBack);
                INTAKE.setPosition(.75);
                Trajectory goForward2 = drive.trajectoryBuilder(goBack.end())
                        .forward(5)
                        .build();
                drive.followTrajectory(goForward2);
                sleep(1000);
                LiftUpForTime(-1, .25);
                LIFT.setPower(0);

                // PICK UP CONE 1
                INTAKE.setPosition(.855);
                //INTAKE.setPosition(.75);
                //LiftUpForTime(-1, .25)
                Trajectory Crab2 = drive.trajectoryBuilder(goForward2.end())
                        .strafeRight(22)
                        .build();
                drive.followTrajectory(Crab2);

                Trajectory goForward3 = drive.trajectoryBuilder(Crab2.end())
                        .forward(25)
                        .build();
                drive.followTrajectory(goForward3);

                Trajectory Strafe = drive.trajectoryBuilder(goForward3.end())
                        .strafeRight(35)
                        .build();
                drive.followTrajectory(Strafe);

                ARM.setPosition(.5);
                sleep(5000);
                //wait for intake to enter into cone
                INTAKE.setPosition(.25);

                //PICK UP CONES FROM STACK--> REPEAT THIS THRICE!
                //WHAT IS THE INTAKE POSITION??
                Trajectory Strafe2 = drive.trajectoryBuilder(Strafe.end())
                        .strafeLeft(58)
                        .build();
                drive.followTrajectory(Strafe2);

                // PLACE CONE 2
                Trajectory goBack2 = drive.trajectoryBuilder(Strafe2.end())
                        .back(3)
                        .build();
                drive.followTrajectory(goBack2);

                Trajectory goForward4 = drive.trajectoryBuilder(goBack2.end())
                        .forward(3)
                        .build();
                drive.followTrajectory(goForward4);

                Trajectory Strafe3 = drive.trajectoryBuilder(goForward4.end())
                        .strafeRight(55)
                        .build();
                drive.followTrajectory(Strafe3);

                // PICK UP CONE 3
                Trajectory Strafe4 = drive.trajectoryBuilder(Strafe3.end())
                        .strafeLeft(56)
                        .build();
                drive.followTrajectory(Strafe4);
                Trajectory Forward9 = drive.trajectoryBuilder(Strafe4.end())
                        .forward(6)
                        .build();
                drive.followTrajectory(Forward9);

                Trajectory Forward8 = drive.trajectoryBuilder(Forward9.end())
                        .back(6)
                        .build();
                drive.followTrajectory(Forward8);

                // PLACE CONE 3
                Trajectory Strafe5 = drive.trajectoryBuilder(Strafe4.end())
                        .strafeRight(50)
                        .build();
                drive.followTrajectory(Strafe5);
                Trajectory Strafe6 = drive.trajectoryBuilder(Strafe5.end())
                        .strafeLeft(85)
                        .build();
                drive.followTrajectory(Strafe6);
                Trajectory Strafe7 = drive.trajectoryBuilder(Strafe6.end())
                        .strafeRight(35)
                        .build();
                drive.followTrajectory(Strafe7);
                Trajectory goBack3 = drive.trajectoryBuilder(Strafe7.end())
                        .back(35)
                        .build();
                drive.followTrajectory(goBack3);

                //ForwardForDistance(0.3, -0.7);

                // if red go to zone 1
                if (redVal > greenVal && redVal > blueVal) {
                */
/*Trajectory Crab3 = drive.trajectoryBuilder(goBack3.end())
                        //SO SUS!! SHOULDN'T IT BE CRABBING??
                        // .forward()
                        .build();
                drive.followTrajectory(Crab3);
*//*

                    //CrabForDistance(0.3, 2.3);
                }
                // if blue go to zone 2 (already there no if statement)

                else if (greenVal > redVal && greenVal > blueVal) {
                */
/*//*
/Trajectory Crab4 = drive.trajectoryBuilder(goBack3.end())
                        //SO SUS!! SHOULDN'T IT BE CRABBING??
                        // .forward()
                        .build();
                drive.followTrajectory(Crab4);*//*

                    //CrabForDistance(0.3, -2.3);
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
}
*/
