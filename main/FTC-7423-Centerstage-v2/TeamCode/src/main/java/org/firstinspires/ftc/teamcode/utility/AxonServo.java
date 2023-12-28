package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class AxonServo  {
    ServoImplEx motor;
    AnalogInput feedback;
    double restPos = 0;
    public AxonServo(ServoImplEx motor, AnalogInput potentiometer){
        this.motor = motor;
        this.feedback = potentiometer;
    }
    public double getPos(){
        return feedback.getVoltage()/3.3;
    }
    public double getPosDegrees(){
        return feedback.getVoltage()/3.3*360;
    }
    public void setPos(double pos){
        motor.setPosition(pos+restPos);
    }
    public void setPosDegrees(double pos){
        motor.setPosition(pos/360 + restPos);
    }
    public void calibrate(){
        //PUT THE SERVO IN THE REST POSITION
        restPos = getPos();
    }
}
