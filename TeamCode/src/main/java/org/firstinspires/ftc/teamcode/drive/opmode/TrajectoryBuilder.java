package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class TrajectoryBuilder extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory toJunction = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(45)
              //  .splineTo(new Vector2d(30, 0), Math.toRadians(90))


//             .forward(40)
//                .strafeRight(60.3)
                // .strafeLeft(60.3)
//                .back(10)


//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(25, -15), 0)
//                                .waitSeconds(3)
//                                .turn(Math.toRadians(45))
//                               .forward(60.3)
                // .strafeRight(25)
//                                .turn(Math.toRadians(90))
//                .splineTo(new Vector2d(-60.3, -62.9), 0)
//                .(Math.toRadians(90))
//                                .forward(60.3)
//                                .turn(Math.toRadians(90))
//                                .forward(25)
//                                .strafeLeft(5)
//                                .waitSeconds(1)
//                                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)), 0)
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(toJunction.end())
                .waitSeconds(3)
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

        drive.followTrajectory(toJunction);
        drive.followTrajectorySequence(wait);


        drive.followTrajectory(backToStart);
        drive.followTrajectory(goBack);
        drive.followTrajectory(backToStart2);
        drive.followTrajectory(toJunction2);




        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(30000);


        if(isStopRequested()) return;


    }
}
