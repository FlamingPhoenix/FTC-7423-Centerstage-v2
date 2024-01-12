package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.ServoSpeedController;

@TeleOp
public class TEST_servospeedd extends OpMode{
    ServoSpeedController servoSpeedController;
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        servoSpeedController = new ServoSpeedController(servo);
    }
    @Override
    public void loop(){
        if(gamepad1.a){
            servoSpeedController.setPositionWithWait(0.5, 2);
        }
        if(gamepad1.b){
            servoSpeedController.setPositionWithWait(0.25, 1.5);
        }
        if(gamepad1.x){
            servoSpeedController.setPositionWithWait(0.75, 2.5);
        }
        if(gamepad1.y){
            servoSpeedController.setPositionWithWait(0, 1);
        }
    }
}
