package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringSystem {
    private final DcMotor intakeRoller, leftConveyor, rightConveyor;
    private final CRServo leftGate, rightGate;

    public ScoringSystem(HardwareMap hw) {
        intakeRoller = hw.get(DcMotor.class, "intake_roller");
        leftConveyor = hw.get(DcMotor.class, "left_conveyor_belt");
        rightConveyor = hw.get(DcMotor.class, "right_conveyor_belt");
        leftGate = hw.get(CRServo.class, "left_intake_gate");
        rightGate = hw.get(CRServo.class, "right_intake_gate");
    }

    public void runLeft(double pwr) {
        leftConveyor.setPower(pwr);
        leftGate.setPower(pwr);
    }

    public void runRight(double pwr) {
        rightConveyor.setPower(pwr);
        rightGate.setPower(pwr);
    }

    public void setRoller(double pwr) {
        intakeRoller.setPower(pwr);
    }

    public void stop() {
        runLeft(0);
        runRight(0);
        setRoller(0);
    }
}
