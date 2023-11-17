package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.Vector;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                           // .strafeTo(new Vector2d(10,-8))
                            .setTangent(0)
                            .splineTo(new Vector2d(40,-40), Math.toRadians(-90))
                            //.lineToY(-24)

                           // .lineToX(30)
                            .build());


         /*   Actions.runBlocking(
                drive.actionBuilder(drive.pose)


                        .lineToX(30)
                        .build());*/





          /*

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.lineToX(30.0)
                            //.turnTo(90)
                            //.lineToX(20)
                            //.strafeTo(10)
                            //.lineToY(10)
                            .strafeTo(new Vector2d(-24,-24))
                            //.splineTo(new Vector2d(24,24),180)
                            .build());
*/





        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)

                            .build());
        } else {
            throw new AssertionError();
        }
    }
}
