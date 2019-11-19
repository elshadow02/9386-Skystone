package org.firstinspires.ftc.teamcode.Config;

public class PIDControl {
    private double kp;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
    private double prevDeriv = 0;
    private double deriv, timeChange = 0;

    public PIDControl(double kp, double kd, double ki){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getOutput(double error, double timeChange){
        deriv = (error - prevError)/timeChange;

        return deriv;
    }
}
