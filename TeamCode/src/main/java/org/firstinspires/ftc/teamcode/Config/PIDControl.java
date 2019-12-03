package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.util.Range;

public class PIDControl {
    private double kp;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
    private double prevDeriv = 0;
    private double deriv, integral, timeChange = 0;

    public PIDControl(double kp, double kd, double ki){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getOutput(double error, double timeChange){
        this.timeChange = timeChange;
        deriv = (error - prevError) / timeChange;
        prevError = error;
        prevDeriv = deriv;
        integral += timeChange * (0.5 * (error + prevError));
        return Range.clip((error * kp) +
                (ki * integral) +
                (kd * deriv), -1, 1);
    }
}
