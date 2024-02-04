package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

// moves servo to locations in increments slower than regular servo movement

public class ServoSpeedController {
    public Servo servo;
    public AxonServo axonServo;
    boolean isAxon = false; //if true, use axon servo, else use servo
    public double speed = 1; // how much movement per chunk
    public double targetPos = 0;
    private double error = 0; // how much further i need to go
    private double delta = 0;
    private double loops = 0; // how many loops it takes to get to target
    //If not axon servo, set isAxon to false
    public ServoSpeedController(Servo servo){
        this.servo = servo;
        this.isAxon = false;
    }
    //If axon servo, set isAxon to true
    public ServoSpeedController(AxonServo servo){
        this.axonServo = servo;
        this.isAxon = true;
    }
    // set speed
    public void setSpeed(double speed){
        this.speed = speed;
        this.delta = speed / 1000;
        this.loops = error / delta;
    }
    //If not axon servo, set target position
    public void setTargetPos(double pos){
        this.targetPos = pos;
        this.error = servo.getPosition() - targetPos;
        this.loops = error / delta;
    }
    // set target position and speed in one function, for axon servos
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
                axonServo.setPosition(axonServo.getPosition() - delta);
            }
        }
    }
    // set target position and speed in one function, for not axon servos
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
                axonServo.setPosition(axonServo.getPosition() - delta);
            }
        }
    }
    // constantly move to target position and speed
    public void loopEvery(){
        if(loops>0){
            if(!isAxon) {
                servo.setPosition(servo.getPosition() - delta);
            }
            else{
                axonServo.setPosition(axonServo.getPosition() - delta);
            }
            loops--;
        }
    }
}
