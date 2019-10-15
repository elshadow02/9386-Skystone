package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import org.firstinspires.ftc.teamcode.Auto.NavigationPositions;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "Nav")
public class NavigationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        NavigationPositions nav = new NavigationPositions();

        int stopButton = 0;

        nav.VuforiaInit();

        waitForStart();

        while (opModeIsActive() && stopButton != 1) {

            nav.resetXYPositions();

            nav.telemetryUpdate();

            if (gamepad1.a){
                stopButton = 1;
                break;
            }

            sleep(7000);
        }
    }
}
