package org.firstinspires.ftc.teamcode.utility;

import java.util.Timer;
import java.util.TimerTask;

public class utils {
    static Timer scheduler = new Timer();
    /**
     * Run a function after a delay asynchronously
     * @param runnable The function to run - can be a lambda
     * @param delay milliseconds to wait before running the function
     */
    public static void setTimeout(Runnable runnable, int delay){
        scheduler.schedule(new TimerTask() {
            @Override
            public void run() {
                runnable.run();
            }
        }, delay);
    }

    /**
     * Clamp a value between a minimum and maximum
     * @param val value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return clamped value
     */
    public static double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
    /**
     * Clamp a value between 0 and 1
     * @param val value to clamp
     * @return clamped value
     */
    public static double clamp(double val){
        return clamp(val, 0.0, 1.0);
    }

    /**
     * Map a value from one range to another
     * @param val value to map
     * @param inMin minimum value of input range
     * @param inMax maximum value of input range
     * @param outMin minimum value of output range
     * @param outMax maximum value of output range
     * @return mapped value
     */
    public static double mapRange(double val, double inMin, double inMax, double outMin, double outMax){
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double average(double ... values ){
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }
    public static double median(double ... values ){
        int middle = values.length/2;
        if (values.length%2 == 1) {
            return values[middle];
        } else {
            return (values[middle-1] + values[middle]) / 2.0;
        }
    }
    public static double mode(double ... values ){
        double mode = values[0];
        int maxCount = 0;
        for (int i = 0; i < values.length; i++) {
            int count = 0;
            for (int j = 0; j < values.length; j++) {
                if (values[j] == values[i]) ++count;
            }
            if (count > maxCount) {
                maxCount = count;
                mode = values[i];
            }
        }
        return mode;
    }
}
