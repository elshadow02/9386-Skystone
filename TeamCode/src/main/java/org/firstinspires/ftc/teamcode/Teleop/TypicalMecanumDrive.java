package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name="Typical Mecanum Drive")
public class TypicalMecanumDrive extends OpMode{

    private EEHardware robot = new EEHardware();

    double kR = 0.95;

    //Define class variables.
    private double forward, strafe, rotate;
    private double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        rotate = gamepad1.right_stick_x;
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;

        rotate *= kR;

        frontLeftSpeed = forward + rotate + strafe;
        frontRightSpeed = forward - rotate - strafe;
        backLeftSpeed = forward + rotate - strafe;
        backRightSpeed = forward - rotate + strafe;

        double max = Math.abs(frontLeftSpeed);
        if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
        if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
        if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);

        if (max > 1) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        robot.frontRight.setPower(frontRightSpeed);
        robot.frontLeft.setPower(frontLeftSpeed);
        robot.backLeft.setPower(backLeftSpeed);
        robot.backRight.setPower(backRightSpeed);
    }

}
