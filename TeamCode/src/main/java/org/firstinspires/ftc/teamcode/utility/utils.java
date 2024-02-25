package org.firstinspires.ftc.teamcode.utility;

public class utils {
    public static void setTimeout(Runnable runnable, int delay){
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e){
                System.err.println(e);
            }
        }).start();
    }

    public static double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
    public static double clamp(double val){
        return clamp(val, 0.0, 1.0);
    }

    public static double mapRange(double val, double inMin, double inMax, double outMin, double outMax){
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
    public static double mapRange(double val, double inMax, double outMax){
        return mapRange(val, 0, inMax, 0, outMax);
    }
    public static double mapRange(double val, double outMax){
        return mapRange(val, 1, outMax);
    }
}
