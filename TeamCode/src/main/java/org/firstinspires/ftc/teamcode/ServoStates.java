package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class ServoStates {
    Servo[] servos;
    String currentState;
    HashMap<String,double[]> states = new HashMap<>();
    public ServoStates(Servo[] servos){
        this.servos = servos;
    }
    public ServoStates(Servo[] servos, double[][] states){
        this.servos = servos;
    }
    public void addState(String name, double[] state){
        states.put(name,state);
    }
    public void setState(String name){
        double[] state = states.get(name);
        currentState = name;
        for(int i  = 0; i< servos.length;i++){
            servos[i].setPosition(state[i]);
        }
    }
    public String getCurrentState(){
        return currentState;
    }
    public double[] getStatePositions(String id){
        return states.get(id);
    }
}
