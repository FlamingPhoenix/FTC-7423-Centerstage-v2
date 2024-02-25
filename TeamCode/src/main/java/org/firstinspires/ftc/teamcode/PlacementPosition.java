package org.firstinspires.ftc.teamcode;

public enum PlacementPosition {
    LEFT(0),
    CENTER(1),
    RIGHT(2);
    int value;
    private PlacementPosition(int value) {
        this.value = value;
    }
    public int getValue() {
        return value;
    }
}