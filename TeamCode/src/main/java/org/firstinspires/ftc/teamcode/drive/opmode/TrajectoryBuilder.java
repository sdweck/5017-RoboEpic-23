//Note: When strafing, robot will go ten less than you put in code
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

        //

    Trajectory toJunction = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(80)
                .build();

    Trajectory goForward = drive.trajectoryBuilder(toJunction.end())
                .forward(5)
                .build();

        LiftUpForDistance(1, -0.1);
        // Intake(0.25)
        sleep(5000);
        /*TrajectorySequence wait2 = drive.trajectorySequenceBuilder(toJunction.end())
                        .waitSeconds(3)
                        .build();*/
        LiftDownForDistance(1,-0.75);

        Trajectory goBack2 = drive. trajectoryBuilder(goForward.end())
                .back(5)
                .build();
      // Trajectory toConeLine = drive.trajectoryBuilder(goBack2.end())
              // .back()MEASURE/TEST DISTANCE!!
    //    Trajectory toJunction1 = drive.trajectoryBuilder(goBack2.end())
              //  .forward()MEASURE/TEST DISTANCE
        //Intake move

        //Go for second cone



        /*Trajectory backToStart = drive.trajectoryBuilder(toJunction.end())
                //.splineTo(new Vector2d(-30, 0), Math.toRadians(90))
                .strafeLeft(75)
                .build();
        Trajectory goBack = drive.trajectoryBuilder(backToStart.end())
                .back(23)
                .build();
        Trajectory backToStart2 = drive.trajectoryBuilder(goBack.end())
                .forward(13)
                .build();
        Trajectory toJunction2 = drive.trajectoryBuilder(backToStart2.end())
                .strafeRight(50)
                .build();
        Trajectory toConeLine2 = drive.trajectoryBuilder(toJunction2.end())
                .forward(2)
                .build();
        Trajectory goBack3 = drive.trajectoryBuilder(toConeLine2.end())
                .back(2)
                .build();*/

        waitForStart();

       //LiftUpForDistance(1, -0.1);
      // Intake(0.25)

      //LiftDownForDistance(1,-0.75);

      drive.followTrajectory(toJunction);
      drive.followTrajectory(goForward);
      drive.followTrajectory(goBack2);
      //drive.followTrajectory(toConeLine);
        //drive.followTrajectory(toJunction1);
//    //Arm Go Completely Down
        //drive.followTrajectory(backToStart);
       // drive.followTrajectory(goBack);
      //drive.followTrajectory(backToStart2);
     //drive.followTrajectory(toJunction2);
      //  drive.followTrajectory(toConeLine2);
       // drive.followTrajectory(goBack3);
        //drive.followTrajectorySequence(wait2);





        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(30000);


        if(isStopRequested()) return;


    }

    private void stopEverything() {
        LIFT.setPower(0);
       // INTAKE.setPower(0);
    }
//Note!! If revolutions negative, the linear slide will go UP
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
