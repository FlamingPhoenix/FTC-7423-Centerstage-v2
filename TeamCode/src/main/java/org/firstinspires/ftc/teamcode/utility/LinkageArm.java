package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LinkageArm {
    Servo motor;
    double bar1len, bar2len,offset,maxLen;

    /**
     * Initialize LinkageArm
     * @param motor Servo for controlling the linkage arm
     * @param bar1len length of the first bar
     * @param bar2len length of the second bar
     * @param offset z-offset between first bar startpoint and second bar endpoint
     */
    public LinkageArm(Servo motor, double bar1len, double bar2len, double offset){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar2len;
        this.offset = offset;
        this.maxLen = bar1len+bar2len-offset;
    }
    public LinkageArm(Servo motor, double bar1len, double bar2len){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar2len;
        this.offset = 0;
        this.maxLen = bar1len+bar2len;
    }
    public LinkageArm(Servo motor, double bar1len){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar1len;
        this.offset = 0;
        this.maxLen = bar1len+bar2len;
    }
    /**
     * Initialize LinkageArm
     * @param motor Servo for controlling the linkage arm
     * @param bar1len length of the first bar
     * @param bar2len length of the second bar
     * @param offset z-offset between first bar startpoint and second bar endpoint
     */
    public LinkageArm(ServoImplEx motor, double bar1len, double bar2len, double offset){
        this.motor = motor;
        this.bar1len = bar1len;
        this.bar2len = bar2len;
        this.offset = offset;
        this.maxLen = bar1len+bar2len-offset;
    }

    /**
     * Set the length of the linkage arm
     * @param len length in the SAME UNIT as bar1len and bar2len
     */
    public void setLen(double len){
        len = Math.min(len+100,maxLen+100);
        double angle = Math.acos(
                (
                    (bar1len*bar1len)
                    +(len*len)
                    -(
                            (bar2len-offset)*(bar2len-offset)
                    )
                )
                        /(2*bar1len*len)
        );
        //servo range 300 degrees
        motor.setPosition((154-Math.toDegrees(angle))/180);
    }
    public double getPosition(){
        return motor.getPosition();
    }
    public void setPosition(double pos){
        motor.setPosition(pos);
    }



    /**
     * Get the length of the linkage arm
     * Don't rely upon this because I am lazy and don't want to do the math and github copilot just wrote this.
     * @return length in the SAME UNIT as bar1len and bar2len
     */
    public double getLen(){//github copilot wrote this, probably doesn't work.
        double angle = motor.getPosition();
        return Math.sqrt(bar1len*bar1len+bar2len*bar2len-2*bar1len*bar2len*Math.cos(angle));
    }
    /**
     * Get the angle of the linkage arm
     * @return angle in radians
     */
    public double calcAngle(double len){
        return Math.acos(bar1len*bar1len+len*len-(bar2len-offset)*(bar2len-offset))/(2*bar1len*len);
    }
}
