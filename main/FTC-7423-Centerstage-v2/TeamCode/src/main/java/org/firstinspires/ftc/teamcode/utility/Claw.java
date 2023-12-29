package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    double[] clawposs;
    double openPos,halfOpenPos, closePos;
    /**
     * Initialize Claw
     * @param claw Servo for controlling the claw
     * @param openPos position of the claw when it is open - drop all pixels
     * @param halfOpenPos position of the claw when it is half open - drop one pixel
     * @param closePos position of the claw when it is closed - pick up pixels
     */
    public Claw(Servo claw, double closePos, double halfOpenPos, double openPos){
        this.claw = claw;
        this.openPos = openPos;
        this.halfOpenPos = halfOpenPos;
        this.closePos = closePos;
        clawposs = new double[]{closePos, halfOpenPos, openPos};
        claw.setPosition(openPos);
    }
    public void open(){
        claw.setPosition(openPos);
    }
    public void halfOpen(){
        claw.setPosition(halfOpenPos);
    }
    public void close(){
        claw.setPosition(closePos);
    }
    public void setPos(double pos){
        claw.setPosition(pos);
    }
    public double getPos(){
        return claw.getPosition();
    }
    public void ezSetPos(int posID){
        if(posID > clawposs.length) throw new IllegalArgumentException("posID is out of bounds");
        claw.setPosition(clawposs[posID]);
    }
    public void ezSetPos(String posName){
        switch(posName){
            case "open":
                open();
                break;
            case "halfOpen":
                halfOpen();
                break;
            case "close":
                close();
                break;
            default:
                throw new IllegalArgumentException("posName is not a valid position name. Valid names are \"open\", \"halfOpen\", and \"close\"");
        }
    }

}
