package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AprilTest extends NewAprilTag{
    @Override
    public void runOpMode() {
        //initAprilTagFront();
        waitForStart();
        alignFront(8,6);
    }
}
