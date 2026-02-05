package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Far Left NoFurther Auto")
public class FarLeftNoFurtherAuto extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private HuskyLens huskyLens;
    private AnalogInput distanceSensor;

    private DcMotor intakeRoller;
    private CRServo rightIntakeGate;
    private DcMotorEx rightConveyorBelt; // Shooter
    private CRServo leftIntakeGate;
    private DcMotorEx leftConveyorBelt; // Shooter

    // --- ROBOT CONSTANTS ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double ROBOT_TRACK_WIDTH_INCHES = 22.0;
    private static final double MAX_VELOCITY = 2000;
    private static final double INTAKE_ROLLER_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;

    // --- ALLIANCE & SEQUENCE ---
    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    private BallColor[] detectedSequence = {BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
    
    private final BallColor[] SEQ_3 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] SEQ_1 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    @Override
    public void runOpMode() {
        initializeHardware();
        boolean selectionMade = true;

        waitForStart();

        if (opModeIsActive() && selectionMade) {
            telemetry.addData("Status", "Scanning while moving...");
            telemetry.update();

            // Step 1: Drive forward and scan.
            double distanceToTag = driveMecanum(-50, 0, 0.7, true);
            turnRobot(-60, 0.7);

            intakeRoller.setPower(0.0);

            if (detectedSequence[0] == BallColor.UNKNOWN) {
                telemetry.addLine("Scanning one last time at stop...");
                telemetry.update();
                detectedSequence = readSequenceFromObelisk(1.0);
            }

            // Return to start line
            //driveMecanum(-distanceToTag, 0, 0.7, false);

            if (detectedSequence[0] != BallColor.UNKNOWN) {
                turnRobot(60, 0.7);

                // FIXED: Use runShootingSequence (looped) instead of runShootingActually (one-off)
                runShootingSequence(4.0, 1450, 0.90);
                
                // Collect Balls Sequence
                intakeRoller.setPower(1.0);
                leftIntakeGate.setPower(0.8);
                rightIntakeGate.setPower(0.8);
                leftConveyorBelt.setPower(-0.8);
                rightConveyorBelt.setPower(-0.8);

                // Curve LEFT
                driveCurve(40.0, 72.0, 0.5);

                // Straight to collect balls
                driveDistance(9, 0.2, false);
                sleep(100);
                stopAllMechanisms();


                driveDistance(-6, 0.2, false);
                driveCurve(-40.0, -72.0, 0.5);
                // Start return motors

                leftConveyorBelt.setPower(-1.0);
                rightConveyorBelt.setPower(-1.0);
                leftIntakeGate.setPower(-0.8);
                rightIntakeGate.setPower(-0.8);
                sleep(500);

                // Final shooting
                runShootingSequence(4.0, 1750, 0.90);

                showLiveStats();
            } else {
                telemetry.addLine("ERROR: No sequence found. Stopping.");
                telemetry.update();
            }
        }
    }

    /**
     * Looped shooting sequence to allow motors to reach target speed in Autonomous.
     */
    public void runShootingSequence(double durationSeconds, double targetVelocity, double thresholdPercent) {
        ElapsedTime shootTimer = new ElapsedTime();
        while (opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            runShootingActually(targetVelocity, thresholdPercent);
            showLiveStats(); // Updates telemetry every loop
            sleep(20);
        }
        stopAllMechanisms();
    }

    public void runShootingActually(double targetVelocity, double powerToUse) {
        leftConveyorBelt.setVelocity(targetVelocity);
        rightConveyorBelt.setVelocity(targetVelocity);

        double threshold = targetVelocity * powerToUse;

        boolean readyLeft = Math.abs(leftConveyorBelt.getVelocity()) >= threshold;
        boolean readyRight = Math.abs(rightConveyorBelt.getVelocity()) >= threshold;

        if (readyLeft && targetVelocity > 100) {
            leftIntakeGate.setPower(GATE_SERVO_POWER);
        } else if (Math.abs(leftConveyorBelt.getVelocity()) < (threshold - 100)) {
            leftIntakeGate.setPower(0);
        }

        if (readyRight && targetVelocity > 100) {
            rightIntakeGate.setPower(GATE_SERVO_POWER);
        } else if (Math.abs(rightConveyorBelt.getVelocity()) < (threshold - 100)) {
            rightIntakeGate.setPower(0);
        }

        if ((readyLeft || readyRight) && targetVelocity > 100) {
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
        } else {
            intakeRoller.setPower(0);
        }
    }

    public void stopAllMechanisms() {
        leftConveyorBelt.setPower(0);
        rightConveyorBelt.setPower(0);
        leftIntakeGate.setPower(0);
        rightIntakeGate.setPower(0);
        intakeRoller.setPower(0);
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
        rightIntakeGate = hardwareMap.get(CRServo.class, "right_intake_gate");
        rightConveyorBelt = hardwareMap.get(DcMotorEx.class, "right_conveyor_belt");
        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
        leftConveyorBelt = hardwareMap.get(DcMotorEx.class, "left_conveyor_belt");

        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeGate.setDirection(DcMotorSimple.Direction.REVERSE);
        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeGate.setDirection(DcMotorSimple.Direction.FORWARD);
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);

        leftConveyorBelt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightConveyorBelt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftConveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightConveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distanceSensor = hardwareMap.get(AnalogInput.class, "ranger");

        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "NOT CONNECTED!");
        } else {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }
        telemetry.update();
    }

    private BallColor[] readSequenceFromObelisk(double scanSeconds) {
        ElapsedTime scanTimer = new ElapsedTime();
        while(opModeIsActive() && scanTimer.seconds() < scanSeconds) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                for (HuskyLens.Block tag : blocks) {
                    if (tag.id == 1) return SEQ_1;
                    if (tag.id == 2) return SEQ_2;
                    if (tag.id == 3) return SEQ_3;
                }
            }
            showLiveStats();
        }
        return SEQ_1;
    }

    private double driveDistance(double distanceInches, double power, boolean scan) {
        return driveMecanum(distanceInches, 0, power, scan);
    }

    private void driveCurve(double leftInches, double rightInches, double power) {
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int leftTarget = (int)(leftInches * countsPerInch);
        int rightTarget = (int)(rightInches * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double ratio = Math.abs(leftInches / rightInches);
        double leftPower, rightPower;

        if (Math.abs(leftInches) > Math.abs(rightInches)) {
            leftPower = power * Math.signum(leftInches);
            rightPower = (power / ratio) * Math.signum(rightInches);
        } else {
            rightPower = power * Math.signum(rightInches);
            leftPower = (power * ratio) * Math.signum(leftInches);
        }

        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);

        while (opModeIsActive()) {
            boolean leftDone = Math.abs(leftFrontDrive.getCurrentPosition()) >= Math.abs(leftTarget);
            boolean rightDone = Math.abs(rightFrontDrive.getCurrentPosition()) >= Math.abs(rightTarget);

            if (leftDone) { leftFrontDrive.setPower(0); leftBackDrive.setPower(0); }
            if (rightDone) { rightFrontDrive.setPower(0); rightBackDrive.setPower(0); }
            if (leftDone && rightDone) break;
            showLiveStats();
            sleep(10);
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double driveMecanum(double forwardInches, double strafeInches, double power, boolean scan) {
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        int lfTicks = (int)((forwardInches + strafeInches) * countsPerInch);
        int rfTicks = (int)((forwardInches - strafeInches) * countsPerInch);
        int lbTicks = (int)((forwardInches - strafeInches) * countsPerInch);
        int rbTicks = (int)((forwardInches + strafeInches) * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            if (scan) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                if (blocks.length > 0) {
                    for (HuskyLens.Block tag : blocks) {
                        if (tag.id == 1) detectedSequence = SEQ_1;
                        else if (tag.id == 2) detectedSequence = SEQ_2;
                        else if (tag.id == 3) detectedSequence = SEQ_3;
                    }
                }
                if (detectedSequence[0] != BallColor.UNKNOWN) break;
            }
            showLiveStats();
            sleep(20); 
        }

        double traveledInches = leftFrontDrive.getCurrentPosition() / countsPerInch;
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return traveledInches;
    }

    private void turnRobot(double angle, double power) {
        double turnDistance = (angle / 360.0) * (Math.PI * ROBOT_TRACK_WIDTH_INCHES);
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int) (turnDistance * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(-targetTicks, targetTicks, -targetTicks, targetTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
            showLiveStats();
            sleep(10);
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setTargetPosition(int lf, int rf, int lb, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);
    }

    private void setDriveRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void showLiveStats() {
        double voltage = distanceSensor.getVoltage();
        double distInches = (voltage * 48.7) - 4.9;

        telemetry.addData("--- SENSORS ---", "");
        telemetry.addData("Distance", "%.1f in", distInches);
        telemetry.addData("Husky ID", detectedSequence[0]);
        telemetry.addData("Shooter Vel (L/R)", "%.0f / %.0f", 
            leftConveyorBelt.getVelocity(), rightConveyorBelt.getVelocity());
        telemetry.update();
    }
}
