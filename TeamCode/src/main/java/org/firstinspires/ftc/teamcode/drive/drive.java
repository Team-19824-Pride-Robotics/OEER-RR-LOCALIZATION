package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import java.util.ArrayList;

@Config
@Autonomous(name="drive")

//@Disabled
public class drive extends LinearOpMode {

    public static double turn = 90;
    // to first pole
    public static double x1 = 20;
    public static double y1 = -.5;
    //to x1
    public static double x2 = 35;
    public static double y2 = 12;
    public static double heading2 = 315;
    //to x2
    public static double x3 = 46;
    public static double y3 = -4;
    public static double heading3 =180;
    //to x3
    public static double x4 = 60;
    public static double y4 = 8;
    // to x4
    public static double x5 = 70;
    public static double y5 = -20;
    public static double heading5 = 90;
    //to x5
    public static double x6 = 105;
    public static double y6 = -25;
    public static double heading6 = 0;
    //to x6
    public static double x7 = 115;
    public static double y7 = -2;
    public static double heading7 = 85;
    //to x7
    public static double x8 = 85;
    public static double y8 = 20;
    public static double heading8 = 170;
    //to x8









    @Override
    public void runOpMode() throws InterruptedException {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    DcMotorEx intake;
    intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");


        while (!isStarted() && !isStopRequested()) {


        }


        if (opModeIsActive()) {



            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    //move to positions 1-3
                    .forward(x1)
                    .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(heading2)))
                    .lineToSplineHeading(new Pose2d(x3, y3, Math.toRadians(heading3)))
                    //run intake motor and then turn it off
                    .UNSTABLE_addTemporalMarkerOffset(0,()->{
                      intake.setPower(.5);
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(0,()->{
                        intake.setPower(0);
                    })
                    //move to positions 4-8
                    .lineTo(new Vector2d(x4, y4))
                    .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(heading5)))
                    .lineToLinearHeading(new Pose2d(x6, y6, Math.toRadians(heading6)))
                    .lineToLinearHeading(new Pose2d(x7, y7, Math.toRadians(heading7)))
                    .lineToLinearHeading(new Pose2d(x8, y8, Math.toRadians(heading8)))

                    .build();

            if (!isStopRequested()) {
                drive.followTrajectorySequence(trajSeq);
            }

//            PoseStorage.currentPose = drive.getPoseEstimate();





        }
    }


}