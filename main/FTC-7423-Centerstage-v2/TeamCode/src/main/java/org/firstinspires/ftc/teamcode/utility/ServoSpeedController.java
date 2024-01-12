package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSpeedController {
    Servo servo;
    public double speed = 1;
    public ServoSpeedController(Servo servo){
        this.servo = servo;
    }
    public void setSpeed(double speed){
        this.speed = speed;
    }
    public void setPositionWithWait(double pos,double seconds){
        //every loop
        //error = curpos-targpos
        //setpos to current position + error*speed
        double error = servo.getPosition()-pos;
        double loopTime = seconds*1000/100;//(seconds * 1000)/10 loops
        double delta = error/100;
        for(int i = 0; i<100; i++){
            servo.setPosition(servo.getPosition() - delta);
            try {
                Thread.sleep((long) loopTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

}
