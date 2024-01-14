package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSpeedController {
    Servo servo;
    AxonServo axonServo;
    boolean isAxon = false;
    public double speed = 1;
    public double targetPos = 0;
    private double error = 0;
    private double delta = 0;
    private double loops = 0;
    public ServoSpeedController(Servo servo){
        this.servo = servo;
        this.isAxon = false;
    }
    public ServoSpeedController(AxonServo servo){
        this.axonServo = servo;
        this.isAxon = true;
    }
    public void setSpeed(double speed){
        this.speed = speed;
    }
    public void setTargetPos(double pos){
        this.targetPos = pos;
    }
    public void initvalues(){
        delta = speed / 1000;
        error = servo.getPosition() - targetPos;
        loops = error / delta;
    }
    public void setPositionWithSpeed(double pos,double speed){
        if(!isAxon) {
            double error = servo.getPosition() - pos;
            double delta = speed / 1000;
            double loops = error / delta;
            for (int i = 0; i < loops; i++) {
                servo.setPosition(servo.getPosition() - delta);
            }
        }
        else{
            double error = axonServo.getReading() - pos;
            double delta = speed / 1000;
            double loops = error / delta;
            for (int i = 0; i < loops; i++) {
                axonServo.setPos(axonServo.getPos() - delta);
            }
        }
    }

    public void setPositionWithSpeed(double pos){
        if(!isAxon) {
            double error = servo.getPosition() - pos;
            double delta = this.speed / 1000;
            double loops = error / delta;
            for (int i = 0; i < loops; i++) {
                servo.setPosition(servo.getPosition() - delta);
            }
        }
        else{
            double error = axonServo.getReading() - pos;
            double delta = this.speed / 1000;
            double loops = error / delta;
            for (int i = 0; i < loops; i++) {
                axonServo.setPos(axonServo.getPos() - delta);
            }
        }
    }
    public void loopEvery(){
        if(loops>0){
            if(!isAxon) {
                servo.setPosition(servo.getPosition() - delta);
            }
            else{
                axonServo.setPos(axonServo.getPos() - delta);
            }
            loops--;
        }
    }
}
