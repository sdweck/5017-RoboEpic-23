package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;



import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class TrajectoryBuilder extends LinearOpMode {
    // Instance variables corresponding to our various motors/servos.
    private Servo INTAKE;
    private DcMotor LIFT;

    final double encRotation = 537.6;

    @Override
    public void runOpMode() {
        INTAKE = hardwareMap.servo.get("INTAKE");
        LIFT = hardwareMap.dcMotor.get("LIFT");
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory toJunction = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(45)
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(toJunction.end())
                .waitSeconds(3)
                .build();
//ARM CODE
        LiftUpForDistance(0.75, 1);
        Intake(0);
        LiftDownForDistance(0.75, 0.5);

        Trajectory goForward = drive.trajectoryBuilder(wait.end())
                .forward(2)
                .build();

        Trajectory goBack2 = drive. trajectoryBuilder(goForward.end())
                .back(2)
                .build();

        Trajectory backToStart = drive.trajectoryBuilder(toJunction.end())
                //.splineTo(new Vector2d(-30, 0), Math.toRadians(90))
                .strafeLeft(47)

                .build();

        Trajectory goBack = drive.trajectoryBuilder(backToStart.end())
                .back(13)
                .build();
        Trajectory backToStart2 = drive.trajectoryBuilder(goBack.end())
                .forward(13)
                .build();
        Trajectory toJunction2 = drive.trajectoryBuilder(backToStart2.end())
                .strafeRight(30)
                .build();


        waitForStart();

        //LiftUpForDistance(0.75, 1);
       // Intake(0);
        //LiftDownForDistance(0.75,0.5);


       drive.followTrajectory(toJunction);
//        drive.followTrajectorySequence(wait);
//        //Arm Code UP
//        drive.followTrajectory(goForward);
//        //Arm Go Down
//        //Arm Go Up
//        drive.followTrajectory(goBack2);
//    //Arm Go Completely Down
//        drive.followTrajectory(backToStart);
//        drive.followTrajectory(goBack);
//        drive.followTrajectory(backToStart2);
//        drive.followTrajectory(toJunction2);




        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(30000);


        if(isStopRequested()) return;


    }
//    private void stopEverything() {
//        LIFT.setPower(0);
//        INTAKE.setPower(0);
//    }
    private void LiftUpForDistance(double power, double revolutions) {
        int denc = (int) Math.round(revolutions * encRotation);

        LIFT.setDirection(DcMotorSimple.Direction.FORWARD);
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LIFT.setTargetPosition(denc);
        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LIFT.setPower(power);
    }

    private void LiftDownForDistance(double power, double revolutions) {
        int denc = (int)Math.round(revolutions * encRotation);

        LIFT.setDirection(DcMotorSimple.Direction.REVERSE);
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LIFT.setTargetPosition(denc);
        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LIFT.setPower(power);
    }
    private void Intake(double position) {
        INTAKE.setPosition(position);
    }
    private void resetEncoders() {
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
