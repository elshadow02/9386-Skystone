package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Teleop.EEHardware;

@TeleOp (name="Drive Test")
public class driveTest extends OpMode{

    private EEHardware robot = new EEHardware();

    double kR = 0.95;

    //Define class variables.
    private double forward, strafe, rotate;
    private double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;
    private double joystickAngle, angleChange, speedAngle;

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //armController.setMaxIntegral(100); // CHECK THIS

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //Assigns joystickAngle as the angle made by the left joystick.
        joystickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        //Defines angleChange variable as the x-value received from the imu.
        angleChange = orientation.thirdAngle;
        //Assigns speedAngle as the difference between joystickAngle and angleChange.
        speedAngle = joystickAngle - angleChange;

        //Make sure that the absolute value of either the y- or x-value of the joystick is greater than 0.1.
        //Because speedAngle will never produce an angle where the sine and cosine of that angle equals 0,
        //we need to make sure that the robot does not move unless the joystick is pressed.
        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {

            forward = Math.sin(speedAngle);
            strafe = Math.cos(speedAngle);

            rotate = kR * gamepad1.right_stick_x;

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
        else{
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }

    }

}
