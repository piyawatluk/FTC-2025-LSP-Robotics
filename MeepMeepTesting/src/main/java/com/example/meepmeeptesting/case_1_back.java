package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class case_1_back {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(98.26, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, -10, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(-12,-25,Math.toRadians(90)), Math.toRadians(-90))
                                .strafeTo(new Vector2d(-13,-50))
                                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(230))
                .splineToLinearHeading(new Pose2d(12,-25,Math.toRadians(90)), Math.toRadians(-90))
                                .strafeTo(new Vector2d(12,-50))
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(230))
                                .strafeToLinearHeading(new Vector2d(0,-20), Math.toRadians(0))
                //.strafeTo(new Vector2d(36,-50))
                //.strafeToLinearHeading(new Vector2d(-55,-10),Math.toRadians(200))
                //.strafeToLinearHeading(new Vector2d(0,0),Math.toRadians(90))
                //.strafeTo(new Vector2d(12,-50))
                //.strafeToLinearHeading(new Vector2d(-55,-10),Math.toRadians(200))
                //.strafeToLinearHeading(new Vector2d(-12,-30),Math.toRadians(90))
                //.strafeTo(new Vector2d(-12,-55))
                //.strafeToLinearHeading(new Vector2d(55,10),Math.toRadians(210))
                //.strafeToLinearHeading(new Vector2d(-55,55),Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}