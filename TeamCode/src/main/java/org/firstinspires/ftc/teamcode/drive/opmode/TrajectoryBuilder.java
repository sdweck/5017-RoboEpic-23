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
@Autonomous(name = "AutoLeft", group = "drive")
public class TrajectoryBuilder extends LinearOpMode{

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

        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new
                SampleMecanumDrive(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // LiftUpForTime(-1, 1.1);
            //LIFT.setPower(0);
            Trajectory goForward1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(38)
                    .build();
            drive.followTrajectory(goForward1);

            Trajectory Crab1 = drive.trajectoryBuilder(goForward1.end())
                    .strafeLeft(22)
                    .build();
            drive.followTrajectory(Crab1);
            //      What it was --> CrabForDistance(0.3, -0.3);
            sleep(2000);

            // read special cone
            while (COLORSENSOR.red() == 0){
                // crab to the right
                telemetry.addData("Red", COLORSENSOR.red());
                telemetry.addData("Green", COLORSENSOR.green());
                telemetry.addData("Blue", COLORSENSOR.blue());
                telemetry.update();
                sleep(5000);
            }

            double redVal = COLORSENSOR.red();
            double greenVal = COLORSENSOR.green();
            double blueVal = COLORSENSOR.blue();

            // backwards to drop loaded cone
            Trajectory goBack = drive.trajectoryBuilder(Crab1.end())
                    .back(10)
                    .build();
            drive.followTrajectory(goBack);

            Trajectory goForward2 = drive.trajectoryBuilder(goBack.end())
                    .forward(10)
                    .build();
            drive.followTrajectory(goForward2);
            //INTAKE.setPosition(.18);
            //sleep(1000);
            // LiftUpForTime(-1, 0.5);

            Trajectory Crab2 = drive.trajectoryBuilder(goForward2.end())
                    .strafeRight(22)
                    .build();
            drive.followTrajectory(Crab2);

            Trajectory goForward3 = drive.trajectoryBuilder(Crab2.end())
                    .forward(37)
                    .build();
            drive.followTrajectory(goForward3);

            Trajectory Strafe = drive.trajectoryBuilder(goForward3.end())
                    .strafeRight(50)
                    .build();
            drive.followTrajectory(Strafe);
            //PICK UP CONES FROM STACK--> REPEAT THIS THRICE!
            //WHAT IS THE INTAKE POSITION??
            Trajectory Strafe2 = drive.trajectoryBuilder(Strafe.end())
                    .strafeLeft(50)
                    .build();
            drive.followTrajectory(Strafe2);

            Trajectory goBack2 = drive.trajectoryBuilder(Strafe2.end())
                    .back(10)
                    .build();
            drive.followTrajectory(goBack2);

            Trajectory goForward4 = drive.trajectoryBuilder(goBack2.end())
                    .forward(10)
                    .build();
            drive.followTrajectory(goForward4);

            Trajectory Strafe3 = drive.trajectoryBuilder(goForward4.end())
                    .strafeRight(50)
                    .build();
            drive.followTrajectory(Strafe3);
            //PICK UP CONES FROM STACK
            Trajectory Strafe4 = drive.trajectoryBuilder(Strafe3.end())
                    .strafeLeft(85)
                    .build();
            drive.followTrajectory(Strafe4);

            //go forward and put in left junction and then back to central position
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
                Trajectory Crab3 = drive.trajectoryBuilder(goBack3.end())
                        //SO SUS!! SHOULDN'T IT BE CRABBING??
                        // .forward()
                        .build();
                drive.followTrajectory(Crab3);

                //CrabForDistance(0.3, 2.3);
            }
            // if blue go to zone 2 (already there no if statement)

            else if (greenVal > redVal && greenVal > blueVal) {
                Trajectory Crab4 = drive.trajectoryBuilder(goBack3.end())
                        //SO SUS!! SHOULDN'T IT BE CRABBING??
                        // .forward()
                        .build();
                drive.followTrajectory(Crab4);
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