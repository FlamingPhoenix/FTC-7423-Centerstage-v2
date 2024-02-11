package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.ServoSpeedController;

import java.util.HashMap;

public class ServoStates {
    Servo[] servos;
    String currentState;
    ServoSpeedController armController;
    HashMap<String,double[]> states = new HashMap<>();
    public ServoStates(Servo[] servos){
        this.servos = servos;
    }
    public ServoStates(Servo[] servos, double[][] states){
        this.servos = servos;
    }
    public void initArm(){
        armController = new ServoSpeedController(servos[3]);
        armController.setSpeed(1);
    }
    public void addState(String name, double[] state){
        states.put(name,state);
    }
    public void setState(String name){
        double[] state = states.get(name);
        currentState = name;
        for(int i  = 0; i< servos.length;i++){
            if(state[i] != -1) {
                servos[i].setPosition(state[i]);
            }
        }
    }

    public void customedSetState(String name){
        if(name=="intermediate"){
            armController.setSpeed(20);
        }else{
            armController.setSpeed(1);
        }
        double[] state = states.get(name);
        servos[0].setPosition(state[0]);
        servos[1].setPosition(state[1]);
        servos[2].setPosition(state[2]);
        armController.setPositionWithSpeed(state[3]);
    }
    public void setStateWithDelay(String name){
        double[] state = states.get(name);
        currentState = name;
        for(int i  = 0; i< servos.length;i++){
            if(state[i] != -1) {
                servos[i].setPosition(state[i]);
            }
        }
    }
    public String getCurrentState(){
        return currentState;
    }
    public double[] getStatePositions(String id){
        return states.get(id);
    }
}
