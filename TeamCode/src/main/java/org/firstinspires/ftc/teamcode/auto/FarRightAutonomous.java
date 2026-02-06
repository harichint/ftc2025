package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Far Right Auto")
public class FarRightAutonomous extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    //    private HuskyLens huskyLens;
    private NormalizedColorSensor colorSensor;
    private DcMotor intakeRoller;
    // Channel 1
    private CRServo rightIntakeGate; //servo
    private DcMotorEx rightConveyorBelt;//shooter
    //Channel 2
    private CRServo leftIntakeGate;
    private DcMotorEx leftConveyorBelt;

    // --- ROBOT CONSTANTS (Tune These!) ---
    private AnalogInput distanceSensor;
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;   // Distance between left and right wheels
    static final double LAUNCH_ANGLE_DEGREES = 60.0;
    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;
    private static final double INTAKE_ROLLER_POWER = 1.0;


    // --- ALLIANCE SELECTION ---
    private enum GoalDirection { LEFT, RIGHT }
    private GoalDirection selectedAlliance = GoalDirection.RIGHT; // Default to Right side goal

    // --- SEQUENCE LOGIC ---
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
//            double distanceToTag = driveMecanum(50, 0, 0.7, true);
//            turnRobot(-60, 0.7);

            intakeRoller.setPower(0.0);

            detectedSequence = SEQ_1;//readSequenceFromObelisk(1.0);

            // Return to start line

            if (detectedSequence[0] != BallColor.UNKNOWN) {
//                turnRobot(60, 0.7);

                // FIXED: Use runShootingSequence (looped) instead of runShootingActually (one-off)
                runShootingSequence(4.0, 1800, 0.90);
                // first Line
                pickUpFromLines(30.0, 52.0, 0.5, 0.7);
//Second line
                pickUpFromLines(54.0, 76.0, 0.5, 0.7);
// third line
                pickUpFromLines(78.0, 100.0, 0.5, 0.7);


                showLiveStats();
            } else {
                telemetry.addLine("ERROR: No sequence found. Stopping.");
                telemetry.update();
            }
        }
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
    public void pickUpFromLines(double leftInches, double rightInches, double powerOn, double powerOff) {
        // Collect Balls Sequence
        intakeRoller.setPower(1.0);
        leftIntakeGate.setPower(0.8);
        rightIntakeGate.setPower(0.8);
        leftConveyorBelt.setPower(-0.8);
        rightConveyorBelt.setPower(-0.8);

        // TODO: Curve should go right
        driveCurve(leftInches, rightInches, powerOn);

        // Straight to collect balls
        driveDistance(7, 0.2);
//                sleep(100);
        stopAllMechanisms();

        driveDistance(-5, 0.7);
        driveCurve(-leftInches, -rightInches, powerOff);
        // Start return motors

        leftConveyorBelt.setPower(-1.0);
        rightConveyorBelt.setPower(-1.0);
        leftIntakeGate.setPower(-1.0);
        rightIntakeGate.setPower(-1.0);
        sleep(200);
        intakeRoller.setPower(1.0);
        leftIntakeGate.setPower(1.0);
        rightIntakeGate.setPower(1.0);
        leftConveyorBelt.setPower(-1.0);
        rightConveyorBelt.setPower(-1.0);
        // Final shooting
        runShootingSequence(4.0, 1800, 0.90);

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

    /**
     * Stops the right shooter components.
     */
    public void stopRightShooter() {
        rightConveyorBelt.setPower(0);
        rightIntakeGate.setPower(0);
    }

    /**
     * Stops the left shooter components.
     */
    public void stopLeftShooter() {
        leftConveyorBelt.setPower(0);
        leftIntakeGate.setPower(0);
    }

    /**
     * Stops all conveyor and gate mechanisms.
     */
    public void stopAllMechanisms() {
        stopRightShooter();
        stopLeftShooter();
        sleep(1000);
        intakeRoller.setPower(0);
    }

    /**
     * Activates the shooter-specific components (conveyor and gate servo).
     */
    public void runRightShooter(BallColor[] seq) {

        if (Arrays.equals(seq, SEQ_1) || Arrays.equals(seq, SEQ_3)) {
            rightConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(800);
            rightIntakeGate.setPower(GATE_SERVO_POWER);
            sleep(1000);
            intakeRoller.setPower(SHOOTER_CONVEYOR_POWER);
        } else if (Arrays.equals(seq, SEQ_2)) {
            rightConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(800);
            rightIntakeGate.setPower(GATE_SERVO_POWER);
        }
    }


    public void runLeftShooter(BallColor[]seq) { //Green
        if (Arrays.equals(seq, SEQ_1) || Arrays.equals(seq, SEQ_3)) {
            leftConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(200);
            leftIntakeGate.setPower(GATE_SERVO_POWER);
        } else if (Arrays.equals(seq, SEQ_2)) {
            leftConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            sleep(150);
            leftIntakeGate.setPower(GATE_SERVO_POWER);
            sleep(1000);
            intakeRoller.setPower(SHOOTER_CONVEYOR_POWER);
        }

    }


    //----------------------------------------------------------------------------------------------
    // INITIALIZATION & HELPER METHODS
    //----------------------------------------------------------------------------------------------

    /** Initializes all hardware and sets motor directions. */
    private void initializeHardware() {
        // Drivetrain
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
        distanceSensor = hardwareMap.get(AnalogInput.class, "ranger");

        // --- Set Motor & Servo Directions ---
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeGate.setDirection(DcMotorSimple.Direction.REVERSE);   // Spins to help feed conveyor
        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to push balls up/out
        leftIntakeGate.setDirection(DcMotorSimple.Direction.FORWARD);   // Spins to help feed conveyor
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE); // Spins to push balls up/out

        // Sensors
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor.setGain(10f);

//        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
//        if (!huskyLens.knock()) {
//            telemetry.addData("FATAL", "HuskyLens not responding!");
//        }
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /** Performs a stationary scan for a sequence tag for a given duration. */
    private BallColor[] readSequenceFromObelisk(double scanSeconds) {
        ElapsedTime scanTimer = new ElapsedTime();
//        while(opModeIsActive() && scanTimer.seconds() < scanSeconds) {
//            HuskyLens.Block[] blocks = huskyLens.blocks();
//            if (blocks.length > 0) {
//                for (HuskyLens.Block tag : blocks) {
//                    if (tag.id == 1) return SEQ_1;
//                    if (tag.id == 2) return SEQ_2;
//                    if (tag.id == 3) return SEQ_3;
//                }
//            }

        sleep(50);
//        }
        telemetry.addLine("ERROR: No sequence tag found. Default sequence loaded.");
        return SEQ_3;
    }

    /** Drives the robot a specific distance in inches using encoders. */
    private void driveDistance(double distanceInches, double power) {
        if (!opModeIsActive()) return;
        driveMecanum(distanceInches, 0, power); // Use the new method for straight movement
    }

    /** Drives the robot diagonally using forward and strafe distances. */
    private void driveMecanum(double forwardInches, double strafeInches, double power) {
        if (!opModeIsActive()) return;

        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

        int lfTicks = (int)((forwardInches + strafeInches) * countsPerInch);
        int rfTicks = (int)((forwardInches - strafeInches) * countsPerInch);
        int lbTicks = (int)((forwardInches - strafeInches) * countsPerInch);
        int rbTicks = (int)((forwardInches + strafeInches) * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);

        // --- FIXED: Uncommented this block ---
        while (opModeIsActive() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
            telemetry.addData("Status", "Driving to target...");
            telemetry.addData("Ticks", "LF: %d, RF: %d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }

        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /** Turns the robot a specific angle using encoders. */
    private void turnRobot(double angle, double power) {
        double turnDistance = (angle / 360.0) * (Math.PI * ROBOT_TRACK_WIDTH_INCHES);
        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int) (turnDistance * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // For a left turn (positive angle), left wheels go forward, right wheels go backward.
        setTargetPosition(targetTicks, -targetTicks, targetTicks, -targetTicks);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            // Optional telemetry
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- LOW-LEVEL MOTOR CONTROL ---
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