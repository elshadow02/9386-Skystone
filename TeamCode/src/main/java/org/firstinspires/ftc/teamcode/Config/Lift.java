package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Lift {

    //TODO: set these variables.
    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;

    public static double G = K_A * 386;

    //TODO: set PID Coefficients.
    public static PIDCoefficients COEFFICIENTS = new PIDCoefficients(
            0,
            0,
            0
    );

    //TODO: set these variables.
    public static double MAX_V = 0;
    public static double MAX_A = 0;
    public static double MAX_J = 0;

    //TODO: Find winch radius and Encoder ticks.
    public static final double WINCH_RADIUS = 1;
    public static final double ENCODER_TICKS_PER_REVOLUTION = 1;

    private MotionProfile profile;
    private PIDFController controller;
    ElapsedTime time;
    private DcMotor motor;

    public Lift (HardwareMap map) {
        motor = map.dcMotor.get("lift");
        time = new ElapsedTime();
        controller = new PIDFController(COEFFICIENTS, K_V, K_A, K_STATIC, (x) -> G);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goToPosition(0);
    }

    private double getPosition () {
        double rotations = motor.getCurrentPosition() / ENCODER_TICKS_PER_REVOLUTION;
        return rotations * 2 * Math.PI * WINCH_RADIUS;
    }

    public void goToPosition (double targetPosition) {
        MotionState start;
        if (profile != null) {
            start = profile.get(time.seconds());
        } else {
            start = new MotionState(getPosition(), 0,0,0);
        }
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                new MotionState(targetPosition, 0, 0, 0),
                MAX_V,
                MAX_A,
                MAX_J
        );
        time.reset();
    }

    public void update () {
        MotionState target = profile.get(time.seconds());
        controller.setTargetPosition(target.getX());
        motor.setPower(controller.update(getPosition(), target.getV(), target.getA()));
    }
}
