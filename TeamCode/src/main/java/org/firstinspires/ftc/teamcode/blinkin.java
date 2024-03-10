package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class blinkin extends OpMode {
    VoltageSensor voltageSensor;
    RevBlinkinLedDriver blinkin;
    @Override
    public void init() {
       blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
       blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
       voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        double voltage = voltageSensor.getVoltage();
        if(voltage < 12.5){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        }
    }
}
