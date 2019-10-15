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
public class DriveTest extends OpMode{

    private EEHardware robot = new EEHardware();

    double kR = 0.95;

    //Define class variables.
    private double forward, strafe, rotate;
    private double frontRightSpeed, frontLeftSpeed, backRightSpeed, backLeftSpeed;
    private double joystickAngle, angleChange, speedAngle;

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        //Assigns joystickAngle as the angle made by the left joystick.
        joystickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        //Defines angleChange variable as the x-value received from the imu.
        angleChange = orientation.thirdAngle;
        //Assigns speedAngle as the difference between joystickAngle and angleChange.
        speedAngle = joystickAngle - angleChange;

        //Gets the rotational value for our drive.
        rotate = kR * gamepad1.right_stick_x;

        double dampner;

        if(Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)){
            dampner = Math.abs(gamepad1.left_stick_x);
        }
        else if (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)){
            dampner = Math.abs(gamepad1.left_stick_y);
        }
        else{
            dampner = Math.abs(gamepad1.left_stick_y);
        }

        //Make sure that the absolute value of either the y- or x-value of the joystick is greater than 0.1.
        //Because speedAngle will never produce an angle where the sine and cosine of that angle equals 0,
        //we need to make sure that the robot does not move unless the joystick is pressed.
        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {

            forward = Math.sin(speedAngle);
            strafe = Math.cos(speedAngle);

            frontRightSpeed = forward - rotate - strafe;
            frontLeftSpeed = forward + rotate + strafe;
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

            robot.frontRight.setPower(frontRightSpeed * dampner);
            robot.frontLeft.setPower(frontLeftSpeed * dampner);
            robot.backLeft.setPower(backLeftSpeed * dampner);
            robot.backRight.setPower(backRightSpeed * dampner);
        }
        else{
            robot.frontRight.setPower(-rotate);
            robot.frontLeft.setPower(rotate);
            robot.backLeft.setPower(rotate);
            robot.backRight.setPower(-rotate);
        }

    }

}
