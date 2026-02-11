package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Near Right Auto")
public class NearRightAutonomous extends LinearOpMode {

    // --- ROBOT CONSTANTS ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;
    static final double OBELISK_SCAN_ANGLE_DEGREES = 60.0;
    private static final double INTAKE_ROLLER_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;
    private static final double SLOWER_POWER = 0.8;

    // --- ALLIANCE & SEQUENCE ---
    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    private final BallColor[] SEQ_1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] SEQ_3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    private BallColor[] detectedSequence = {BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private HuskyLens huskyLens;
    private AnalogInput distanceSensor;
    private DcMotor intakeRoller;
    private CRServo rightIntakeGate, leftIntakeGate;
    private DcMotorEx rightConveyorBelt, leftConveyorBelt;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

//        if (opModeIsActive()) {
            // Step 1: Drive backward.
            driveMecanum(-40, 0, 0.70, false);

            // Step 2: Turn to read sequence
//            turnRobot(-100.0, 0.85);

            // Step 3: Scan obelisk
            detectedSequence = SEQ_1;//readSequenceFromObelisk(1.0);
        sleep(100);

        if (detectedSequence[0] != BallColor.UNKNOWN) {
                // Step 4: Turn towards goal (Resulting in +10 degrees from start)
//                turnRobot(110.0, 0.95);

                // Initial shooting
                runShootingSequence(2.5, 1250, 0.90, detectedSequence);

                // Reset to 0 baseline for relative turns
//                turnRobot(-10.0, 0.95);

                // Line 1: Straight (44 inches)
                pickUpLineShoot(113.0, 96, 39, detectedSequence);

                // Line 2: Curve (24" from Line 1)
//                pickUpLineShoot(150.0, 75, 86, 1300, detectedSequence);
//
//                // Line 3: Curve (24" from Line 2)
//                pickUpLineShoot(165.0, 92, 110, 1300, detectedSequence);
                turnRobot(-OBELISK_SCAN_ANGLE_DEGREES, 0.7); // Turn left for right side Alliance

                // Final Park
                driveDistance(-30, 0.4, false);
//            } else {
//                telemetry.addLine("ERROR: No sequence found. Stopping.");
//                telemetry.update();
//            }
        }
    }

//    /**
//     * Handles collection cycle using relative encoder turns.
//     * Ensures robot returns to the goal-facing orientation (+10 deg) before shooting.
//     */
//    public void pickUpLineShoot(double angleBack, double leftInches, double rightInches, double shootVelocity, BallColor[] seq) {
//        // 1. Turn to balls from 0 baseline
//        turnRobot(angleBack, 0.95);
//
//        // 2. Drive forward to intake
//        if (leftInches == rightInches) driveMecanum(leftInches, 0, 0.6, true);
//        else driveCurve(leftInches, rightInches, 0.6, true);
//
//        stopAllMechanisms();
//        sleep(20);
//
//        // 3. Return trip
//        if (leftInches == rightInches) driveMecanum(-leftInches, 0, 0.9, false);
//        else driveCurve(-leftInches, -rightInches, 0.9, false);
//
//        // 4. Turn to face goal (Goal is at +10 deg, we are at angleBack deg)
//        // Relative turn = 10 - angleBack
//        turnRobot(10.0 - angleBack, 0.95);
//        sleep(50);
//
//        // 5. Shoot
//        runShootingSequence(1.4, shootVelocity, 0.85, seq);
//
//        // 6. Return to 0 baseline for the next trip
//        turnRobot(-10.0, 0.95);
//    }


    public void pickUpLineShoot(double angleBack, double angleForward, double backwardInches, BallColor[] detectedSequence)  {
        telemetry.addLine("Step 6a: turning right to collect balls...");
        telemetry.update();
        turnRobot(angleBack, 0.7);// assuming the ball collector will pull all the balls within its path
        telemetry.addLine("Step 6b: drive towards the balls..");
        telemetry.update();
        driveMecanum(backwardInches, 0, 0.3, true); //Drive 36 inches fwd // for next set of balls  add + 24 and for the next one  + 24
        driveMecanum(-backwardInches, 0, 0.7, false); //Drive 36 inches back

        turnRobot(-angleForward, 0.7);// turn towards goalpost //make it +90 if rotation is not correct.
        telemetry.addLine("Step 10: Shoot 3 balls...");
        telemetry.update();
        runShootingSequence(3.0, 1250, 0.90, detectedSequence);


    }


    public void runShootingSequence(double durationSeconds, double targetVelocity, double thresholdPercent, BallColor[] seq) {
        ElapsedTime shootTimer = new ElapsedTime();
        while (opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            runShootingActually(targetVelocity, thresholdPercent);
            showLiveStats();
            sleep(10);
        }
        stopAllMechanisms();
    }


    public void runShootingActually(double targetVelocity, double powerToUse) {
        // 3. Apply Velocity
        leftConveyorBelt.setVelocity(targetVelocity);
        rightConveyorBelt.setVelocity(targetVelocity);

        // 4. Trigger Gates ONLY when belts reach 90% of target velocity
        // We lowered this from 95% to 90% to help the motors fire more reliably at lower powers
        boolean readyToShootLeft =  Math.abs(leftConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse);
        boolean readyToShootRight = Math.abs(rightConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse) + 100;

        // Combined check: Both must be at speed, and the target must be valid
        // Logic for Left side
// We use 0.90 to trigger opening, but we could stay open as long as it's > 0.85
// to prevent flickering when the pixel puts load on the motor.
        if (readyToShootLeft && targetVelocity > 100) {
            leftIntakeGate.setPower(GATE_SERVO_POWER);
        } else {
            // Only close the gate if it drops significantly below target
            if (Math.abs(leftConveyorBelt.getVelocity()) < (targetVelocity * (powerToUse - 0.05))) {
                leftIntakeGate.setPower(0);
            }
        }

// Logic for Right side
        if (readyToShootRight && targetVelocity > 100) {
            rightIntakeGate.setPower(GATE_SERVO_POWER);
        } else {
            if (Math.abs(rightConveyorBelt.getVelocity()) < (targetVelocity * (powerToUse - 0.05))) {
                rightIntakeGate.setPower(0);
            }
        }

        // Intake Roller logic (only run if at least one side is ready to feed)
        if ((readyToShootLeft || readyToShootRight) && targetVelocity > 100) {
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
        } else {
            intakeRoller.setPower(0);
        }
        // Telemetry to help the driver see why it may not be shooting yet
        telemetry.addData("Target Velo", "%.0f", targetVelocity);
        telemetry.addData("L-Vel Actual", "%.0f", leftConveyorBelt.getVelocity());
        telemetry.addData("R-Vel Actual", "%.0f", rightConveyorBelt.getVelocity());
        telemetry.addData("Ready to Shoot", (readyToShootLeft && readyToShootRight) ? "YES" : "SPINNING UP...");

    }

    private void showLiveStats() {
        double voltage = distanceSensor.getVoltage();
        double distInches = (voltage * 48.7) - 4.9;
        telemetry.addData("Dist", "%.1f in", distInches);
        telemetry.addData("ID", detectedSequence[0]);
        telemetry.addData("Velo L/R", "%.0f / %.0f", 
            leftConveyorBelt.getVelocity(), rightConveyorBelt.getVelocity());
        telemetry.update();
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRoller = hardwareMap.get(DcMotor.class, "intake_roller");
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeGate = hardwareMap.get(CRServo.class, "right_intake_gate");
        rightIntakeGate.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");

        rightConveyorBelt = hardwareMap.get(DcMotorEx.class, "right_conveyor_belt");
        leftConveyorBelt = hardwareMap.get(DcMotorEx.class, "left_conveyor_belt");
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);

        leftConveyorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightConveyorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor = hardwareMap.get(AnalogInput.class, "ranger");
        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        stopAllMechanisms();
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
            sleep(20);
        }
        return SEQ_1;
    }

    private void driveDistance(double distanceInches, double power, boolean intake) {
        driveMecanum(distanceInches, 0, power, intake);
    }

    private void driveCurve(double leftInches, double rightInches, double power, boolean intake) {
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int lfTicks = (int)(leftInches * countsPerInch);
        int rfTicks = (int)(rightInches * countsPerInch);
        int lbTicks = (int)(leftInches * countsPerInch);
        int rbTicks = (int)(rightInches * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (intake) {
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
            leftConveyorBelt.setPower(-0.8);
            rightConveyorBelt.setPower(-0.8);
        }

        setDrivePower(power);
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
            showLiveStats();
        }
        setDrivePower(0);
        stopAllMechanisms();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveMecanum(double backwardInches, double strafeInches, double power, boolean intake) {
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int lfTicks = (int)((backwardInches + strafeInches) * countsPerInch);
        int rfTicks = (int)((backwardInches - strafeInches) * countsPerInch);
        int lbTicks = (int)((backwardInches - strafeInches) * countsPerInch);
        int rbTicks = (int)((backwardInches + strafeInches) * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (intake) {

            leftConveyorBelt.setPower(-INTAKE_ROLLER_POWER);
            rightConveyorBelt.setPower(-INTAKE_ROLLER_POWER);
            sleep(100);
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
            rightIntakeGate.setPower(SLOWER_POWER);
            leftIntakeGate.setPower(SLOWER_POWER);
        }

        setDrivePower(power);
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
            showLiveStats();
        }
        setDrivePower(0);
        if (intake) {
            stopAllMechanisms();
        }
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnRobot(double angle, double power) {
        double turnDistance = (angle / 360.0) * (Math.PI * ROBOT_TRACK_WIDTH_INCHES);
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int) (turnDistance * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(targetTicks, -targetTicks, targetTicks, -targetTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            showLiveStats();
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAllMechanisms() {
        leftConveyorBelt.setPower(0);
        rightConveyorBelt.setPower(0);
        leftIntakeGate.setPower(0);
        rightIntakeGate.setPower(0);
        intakeRoller.setPower(0);
    }

    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void setDriveRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    private void setTargetPosition(int lf, int rf, int lb, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);
    }
}
