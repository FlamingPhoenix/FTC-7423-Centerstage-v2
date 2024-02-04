package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class AxonServo{
    ServoImplEx motor;
    AnalogInput feedback;
    double restPos = 0;
    double zeroPosOffset = 0;
    private double range = 255;

    /**
     * Initialize AxonServo
     * @param motor servo for controlling the servo
     * @param potentiometer analog feedback input
     */
    public AxonServo(ServoImplEx motor, AnalogInput potentiometer){
        this.motor = motor;
        this.feedback = potentiometer;
    }
    public AxonServo(ServoImplEx motor){
        this.motor = motor;
    }
    /**
     * Initialize AxonServo
     * @param motor servo for controlling the servo
     * @param potentiometer analog feedback input
     */
    public AxonServo(Servo motor, AnalogInput potentiometer) {
        this.motor = (ServoImplEx) motor;
        this.feedback = potentiometer;
    }
    /**
     * Get the corresponding position the servo should be in in order for the servo to be at a certain angle (with respect to position 0 on the servo)
     * @param angle the angle with respect to position 0 on the servo
     * @return the corresponding servo position
     */
    public double getPosFromAngle(double angle){return angle/this.range;}
    public void setRange(double range){
        this.range = range;
    }
    public double getRange(){
        return range;
    }
    /**
     * Reset the servo to the restPos
     */
    public void reset(){
        motor.setPosition(restPos);
    }
    /**
     * set the zeroPos - the servo position at which the arm is parallel to the ground
     */
    public void setZeroPosOffset(double offset){
        zeroPosOffset = offset;
    }
    /**
     * Get the position of the servo in radians
     * @return position in radians
     */
    public double getRestPos() {return restPos;}
    public double getReading(){
        return feedback.getVoltage()/3.3*Math.toRadians(360);
    }
    /**
     * Get the position of the servo in degrees
     * @return position in degrees
     */
    public double getReadingDegrees(){
        return feedback.getVoltage()/3.3*360;
    }
    /**
     * Get the position of the servo between 0 and 1
     * @return position between 0 and 1
     */
    public double getPos() {
        return motor.getPosition();
    }
    /**
     * Set the position of the servo between 0 and 1
     * @param pos position between 0 and 1
     */
    public void setPos(double pos){
        motor.setPosition(pos);
    }
    /**
     * Set the position of the servo in degrees with respect to the ground
     * @param pos position in degrees
     */
    public void setPosDegrees(double pos){
        motor.setPosition(pos/range + zeroPosOffset);
    }
    /**
     * Set the position of the servo in radians with respect to the ground
     * @param pos position in radians
     */
    public void setPosRadians(double pos){
        motor.setPosition(Math.toDegrees(pos)/range + zeroPosOffset);
    }

    /**
     * Calibrate the servo
     * sets the current position to be "0"
     */
    public void calibrate(){
        //PUT THE SERVO IN THE REST POSITION
        restPos = getReading();
    }
    /**
     * Calibrate the servo
     * sets the current position to be "restPos"
     * @param restPos position of the servo when it is at rest
     */
    public void calibrate(double restPos){
        this.restPos = restPos;
    }


}
