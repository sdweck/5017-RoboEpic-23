package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60.3,-34.9,0))
                                .splineTo(new Vector2d(0, -34.9), Math.toRadians(90))
//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(25, -15), 0)
//                                .waitSeconds(3)
//                                .turn(Math.toRadians(45))
//                               .forward(60.3)
                               // .strafeRight(25)
//                                .turn(Math.toRadians(90))
                                .waitSeconds(5)
                                .splineTo(new Vector2d(-60.3, -62.9), 0)
//                                .turn(Math.toRadians(90))
//                                .forward(60.3)
//                                .turn(Math.toRadians(90))
//                                .forward(25)
//                                .strafeLeft(5)
//                                .waitSeconds(1)
//                                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)), 0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
