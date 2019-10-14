/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
    FTC Team 9386 Elmer and Elsie Robotics Skystone
 */
package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * Main Hardware Class
 *
 * Made by Ethan. Skystone 2019-20 Season
 * 
 *
 * Motors, Servos, and other hardware devices
 *
 *
 * Rev IMU:                                 "imu"
 *
 * Motor channel:  Front right drive motor: "frontRight"
 * Motor channel:  Front left drive motor:  "frontLeft"
 * Motor channel:  Back left drive motor:   "backLeft"
 * Motor channel:  Back right drive motor:  "backRight"
 */
public class Robot
{
    /* Public OpMode members. */
    public BNO055IMU imu = null;

    // WHEEL CONFIG: frontRight = FrontRight, frontLeft = FrontLeft
    public DcMotor motor1      = null;
    public DcMotor motor2      = null;
    public DcMotor motor3      = null;
    public DcMotor motor4      = null;

    public DcMotor motor5      = null;
    public DcMotor motor6      = null;
    public DcMotor motor7      = null;
    public DcMotor motor8      = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

//    public static final double TRACK_WIDTH = 16;
//    public static final double WHEEL_BASE = 16;
    public static final double WHEEL_RADIUS = 2;
    public static final double GEAR_RATIO = 1;

    /* Constructor */
    public Robot(){

    }

    public void setImu(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Motors
        motor1 = hwMap.get(DcMotor.class, "fR");
        motor2 = hwMap.get(DcMotor.class, "fL");
        motor3 = hwMap.get(DcMotor.class, "bL:");
        motor4 = hwMap.get(DcMotor.class, "bR");

        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void DriveMode(DcMotor.RunMode mode){
        motor1.setMode(mode);
        motor2.setMode(mode);
        motor3.setMode(mode);
        motor4.setMode(mode);
    }

    public void DrivePower (double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }

    public void DrivePower (double leftPower, double rightPower){
        motor1.setPower(rightPower);
        motor2.setPower(leftPower);
        motor3.setPower(leftPower);
        motor4.setPower(rightPower);
    }

    public void DrivePower (double power1, double power2, double power3, double power4){
        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);
        motor4.setPower(power4);
    }

    public void Turn (double angle){

    }

    private double motorTicksToWheelPosition (int ticks) {
        double rotations = ticks / (motor1.getMotorType().getTicksPerRev() * GEAR_RATIO);
        return rotations * 2 * Math.PI * WHEEL_RADIUS;
    }

    @NotNull
    //@Override
    public List<Double> getWheelPositions () {
        return Arrays.asList(
                motorTicksToWheelPosition(motor1.getCurrentPosition()),
                motorTicksToWheelPosition(motor2.getCurrentPosition()),
                motorTicksToWheelPosition(motor3.getCurrentPosition()),
                motorTicksToWheelPosition(motor4.getCurrentPosition())
        );
    }
}