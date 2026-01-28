package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private final DcMotor lf, rf, lb, rb;
    static final double TICKS_PER_INCH = 537.7 / (3.78 * Math.PI);

    public DriveTrain(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveMecanum(double forward, double strafe, double power) {
        int lfTarget = (int)((forward + strafe) * TICKS_PER_INCH);
        int rfTarget = (int)((forward - strafe) * TICKS_PER_INCH);
        // ... set targets and RUN_TO_POSITION logic ...
    }

    public void turn(double degrees, double power) {
        // ... Turning logic ...
    }

    public boolean isBusy() {
        return lf.isBusy() || rf.isBusy();
    }
}
