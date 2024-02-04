package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.ServoSpeedController;
// this is a test program for the arm, it gives servo position and target position
@TeleOp
public class armTest extends OpMode {
    ServoSpeedController ssc;
    @Override
    public void init() {
        ssc = new ServoSpeedController(hardwareMap.servo.get("armservo"));
        ssc.setSpeed(1);
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            ssc.setTargetPos(0.2);
        }
        if(gamepad1.b){
            ssc.setTargetPos(0.4);
        }
        ssc.loopEvery();
        telemetry.addData("pos", ssc.servo.getPosition());
        telemetry.addData("target", ssc.targetPos);
        telemetry.update();
    }
}
