package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoRelease {
    double releasePos,restPos;
    Servo servo;
    public ServoRelease(Servo servo, double releasePos, double restPos){
        this.servo = servo;
        this.releasePos = releasePos;
        this.restPos = restPos;
        servo.setPosition(restPos);
    }
    public void release(){
        servo.setPosition(releasePos);
    }
    public void rest(){
        servo.setPosition(restPos);
    }
    public void setPosition(double pos){
        servo.setPosition(pos);
    }
    public double getPosition(){
        return servo.getPosition();
    }

}
