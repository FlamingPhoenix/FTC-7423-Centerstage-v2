package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LinkageArm {
    Servo motor;
    double bar1len, bar2len,offset;
    public LinkageArm(Servo motor, double bar1len, double bar2len, double offset){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar2len;
        this.offset = offset;
    }
    public LinkageArm(Servo motor, double bar1len){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar1len;
        this.offset = 0;
    }
    public LinkageArm(ServoImplEx motor, double bar1len, double bar2len, double offset){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar2len;
        this.offset = offset;
    }
    public void setLen(double len){
        double angle = Math.acos(bar1len*bar1len+len*len-(bar2len-offset)*(bar2len-offset))/(2*bar1len*len);
        motor.setPosition(angle);
    }
    public double getLen(){//github copilot wrote this, probably doesn't work.
        double angle = motor.getPosition();
        return Math.sqrt(bar1len*bar1len+bar2len*bar2len-2*bar1len*bar2len*Math.cos(angle));
    }
    public double calcAngle(double len){
        return Math.acos(bar1len*bar1len+len*len-(bar2len-offset)*(bar2len-offset))/(2*bar1len*len);
    }
}
