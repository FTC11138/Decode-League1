package org.firstinspires.ftc.teamcode.util.wrappers;

public class RE_DcMotorExParams {
    public int minPosition, maxPosition, slow;
    public double maxPower,minPower, upRatio, downRatio, slowRatio;

    public RE_DcMotorExParams(int minPos, int maxPos, int slow, double maxPow, double minPow, double upRat, double downRat, double slowRat){
        this.minPosition = minPos;
        this.maxPosition = maxPos;
        this.slow = slow;
        this.maxPower = maxPow;
        this.minPower = minPow;
        this.upRatio = upRat;
        this.downRatio = downRat;
        this.slowRatio = slowRat;
    }
}
