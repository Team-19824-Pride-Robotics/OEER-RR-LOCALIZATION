package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import java.util.ArrayList;



@Config
@Autonomous(name="PracticeAuto")

//@Disabled
public class PracticeAuto extends LinearOpMode {

    public static double turn = 90;
    //first point
    public static double x1 = 30;
    public static double y1 = -.5;
    //second point
    public static double x2 = 40;
    public static double y2 = -20;
    public static double heading2 = 0;
    //third point
    public static double x3 = 115;
    public static double y3 = -20;
    public static double heading3 = 0;
    //fourth point
    public static double x4 = 115;
    public static double y4 = 0;
    //fifth point
    public static double x5 = 80;
    public static double y5 = 0;
    public static double heading5 = 180;
    //sixth point
    public static double x6 = 80;
    public static double y6 = 10;
    public static double heading6 = 0;
    //seventh point
    public static double x7 = 30;
    public static double y7 = 10;
    public static double heading7 = 125;



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

                    .forward(x1)
                    .splineTo(new Vector2d(x2,y2), Math.toRadians(0))
                    .forward(x3)
                    .strafeLeft(x4)
                    .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(heading5)))
                    .back(x4)
                    .forward(x5)
                    .back(x4)
                    .forward(x5)
                    .strafeLeft(x6)
                    .lineToLinearHeading(new Pose2d(x7, y7, Math.toRadians(heading7)))


                    .build();

            if (!isStopRequested()) {
                drive.followTrajectorySequence(trajSeq);
            }

//            PoseStorage.currentPose = drive.getPoseEstimate();


        }
    }

}



