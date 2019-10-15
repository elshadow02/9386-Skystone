/*
    FTC Team 9386 Elmer and Elsie Robotics Skystone
 */
package org.firstinspires.ftc.teamcode.Auto;

/**
 * Main Coordinate Position Class.
 *
 * Made by Ethan. Skystone 2019-20 Season
 */
public class CoordinatePosition
{
    public double XCoordinate, YCoordinate, ZCoordinate;

    /* Constructor */
    public CoordinatePosition(){

    }

    public void setXCoordinate(double XCoordinate) {
        this.XCoordinate = XCoordinate;
    }

    public void setYCoordinate(double YCoordinate) {
        this.YCoordinate = YCoordinate;
    }

    public void setZCoordinate(double ZCoordinate) {
        this.ZCoordinate = ZCoordinate;
    }

    public void setXYZ(double XCoordinate, double YCoordinate, double ZCoordinate){
        this.XCoordinate = XCoordinate;
        this.YCoordinate = YCoordinate;
        this.ZCoordinate = ZCoordinate;
    }

    public void setXY(double XCoordinate, double YCoordinate){
        this.XCoordinate = XCoordinate;
        this.YCoordinate = YCoordinate;
    }

    public double getXCoordinate(){
        return this.XCoordinate;
    }

    public double getYCoordinate(){
        return this.YCoordinate;
    }

    public double getZCoordinate(){
        return this.ZCoordinate;
    }
}