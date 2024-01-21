package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoDegreeController {
    Servo servo;
    double range, min, max;
    /**
     * Initialize ServoDegreeController
     * @param servo servo for controlling the servo
     * @param range range of the servo(degrees)
     * @param min minimum position of the servo
     * @param max maximum position of the servo
     */
    public ServoDegreeController(Servo servo,double range, double min, double max){
        this.servo = servo;
        this.range = range;
        this.min = min;
        this.max = max;
    }
    public ServoDegreeController(Servo servo, double range, double zeroPos){
        this.servo = servo;
        this.range = range;
        this.min = zeroPos*range-range/2;
        this.max = zeroPos*range+range/2;
    }
    /**
     * Initialize ServoDegreeController
     * @param servo servo for controlling the servo
     * @param range range of the servo(degrees)
     */
    public ServoDegreeController(Servo servo,double range) {
        this.servo = servo;
        this.range = range;
        this.min = 0;
        this.max = 1;
    }
    public void setPositionDegrees(double pos){
        servo.setPosition((pos/range)*(max-min)+min);
    }
    public void setPosition(double pos){
        servo.setPosition(pos);
    }
    public double getPosition(){
        return servo.getPosition();
    }
    public double getPositionDegrees(){
        return (servo.getPosition()-min)/(max-min)*range;
    }
    public void setPositionRadians(double pos){
        servo.setPosition((Math.toDegrees(pos)/Math.toDegrees(range))*(max-min)+min);
    }
    public double getPositionRadians(){
        return Math.toRadians((servo.getPosition()-min)/(max-min)*range);
    }
}
