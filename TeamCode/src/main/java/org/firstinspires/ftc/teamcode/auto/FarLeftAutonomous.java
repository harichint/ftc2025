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

import java.util.Arrays;

@Autonomous(name = "Far Left Auto")
public class FarLeftAutonomous extends LinearOpMode {

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

    // State for shooting latch
    private boolean isShootingLatch = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        boolean selectionMade = true;

        waitForStart();

        if (opModeIsActive() && selectionMade) {
            telemetry.addData("Status", "Scanning while moving...");
            telemetry.update();

            // Step 1: Drive forward and scan.
            double distanceToTag = driveMecanum(60, 0, 0.7, true);
            
            telemetry.addData("Final Sequence", Arrays.toString(detectedSequence));
            telemetry.addData("Distance to Tag", "%.2f in", distanceToTag);
            telemetry.update();
            
            if (detectedSequence[0] == BallColor.UNKNOWN) {
                telemetry.addLine("Scanning one last time at stop...");
                telemetry.update();
                detectedSequence = readSequenceFromObelisk(1.5);
            }

//            sleep(200);

            // Return to start line
            driveMecanum(-distanceToTag, 0, 0.7, false);

            if (detectedSequence[0] != BallColor.UNKNOWN) {
                turnRobot(40, 0.5); 

                // Increased shooting to 4 seconds to ensure all balls clear
                runShootingSequence(4.0);

//                turnRobot(-40, 0.5);

                driveDistance(12, 0.5, false);

                double sideTurnAngle = 90;
                turnRobot(sideTurnAngle, 0.5);

                intakeRoller.setPower(0.8);
                leftIntakeGate.setPower(0.8);
                leftConveyorBelt.setPower(-0.8);
                rightConveyorBelt.setPower(-0.8);

                driveDistance(36, 0.5, false);
                driveDistance(6, 0.2, false);      
                sleep(200);
                stopAllMechanisms();

                driveDistance(-42, 0.6, false);
                turnRobot(-sideTurnAngle, 0.5); 
                driveDistance(-12, 0.5, false);

                turnRobot(40, 0.5);

                // Final shooting for 4 seconds
                runShootingSequence(4.0);
                showLiveStats();
            } else {
                telemetry.addLine("ERROR: No sequence found. Stopping.");
                telemetry.update();
            }
        }
    }


    public void runShootingActually(double targetVelocity, double powerToUse, boolean allowGates) {
        leftConveyorBelt.setVelocity(targetVelocity);
        rightConveyorBelt.setVelocity(targetVelocity);

        // Check if motors are within threshold
        boolean readyLeft = Math.abs(leftConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse);
        boolean readyRight = Math.abs(rightConveyorBelt.getVelocity()) >= (targetVelocity * powerToUse);

        // Latch logic: if we are allowed to shoot and motors are ready, OPEN GATES and KEEP THEM OPEN
        if (allowGates && (isShootingLatch || (readyLeft && readyRight)) && targetVelocity > 100) {
            isShootingLatch = true; // Stay in shooting mode
            leftIntakeGate.setPower(GATE_SERVO_POWER);
            rightIntakeGate.setPower(GATE_SERVO_POWER);
            intakeRoller.setPower(INTAKE_ROLLER_POWER);
        } else {
            // Only reset latch if we aren't supposed to be shooting anymore
            if (!allowGates) isShootingLatch = false;
            
            leftIntakeGate.setPower(0);
            rightIntakeGate.setPower(0);
            intakeRoller.setPower(0);
        }
        
        telemetry.addData("Shooting", isShootingLatch ? "FIRING" : "SPINNING UP");
        telemetry.addData("Target", "%.0f", targetVelocity);
        telemetry.addData("L-Actual", "%.0f", leftConveyorBelt.getVelocity());
        telemetry.addData("R-Actual", "%.0f", rightConveyorBelt.getVelocity());
        telemetry.update();
    }


    public void runShootingSequence(double durationSeconds) {
        ElapsedTime shootTimer = new ElapsedTime();
        isShootingLatch = false; // Reset latch for new sequence
        
        double distInches = (distanceSensor.getVoltage() * 48.7) - 4.9;
        
        // Safety: ensure distance is within reasonable bounds
        if (distInches < 5 || distInches > 100 || Double.isNaN(distInches)) {
            distInches = 30.0; 
        }

        double powerRatio;
        if (distInches < 50) {
            powerRatio = (distInches * 0.0015) + 0.67;
        } else {
            powerRatio = (distInches * 0.00238) + 0.643;
        }

        double targetVelocity = Math.min(powerRatio * MAX_VELOCITY, MAX_VELOCITY);
        // Lowered firing threshold to 70% to be more reliable
        double firingThreshold = 0.70;

        // Loop for the specified duration
        while (opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            // Increased pre-spin to 0.8s for better torque
            runShootingActually(targetVelocity, firingThreshold, shootTimer.seconds() > 0.8);
            sleep(20);
        }
        
        stopAllMechanisms();
    }

    public void stopRightShooter() {
        rightConveyorBelt.setPower(0);
        rightIntakeGate.setPower(0);
    }

    public void stopLeftShooter() {
        leftConveyorBelt.setPower(0);
        leftIntakeGate.setPower(0);
    }

    public void stopAllMechanisms() {
        stopRightShooter();
        stopLeftShooter();
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

        // Using default PID coefficients for stability
        distanceSensor = hardwareMap.get(AnalogInput.class, "ranger");

        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "NOT CONNECTED! Check cable.");
        } else {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            telemetry.addData(">>", "HuskyLens Initialized.");
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
            telemetry.addData("Scanning", "%.1f / %.1f", scanTimer.seconds(), scanSeconds);
            telemetry.update();
        }
        return SEQ_1;
    }

    private double driveDistance(double distanceInches, double power, boolean scan) {
        return driveMecanum(distanceInches, 0, power, scan);
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
                if (detectedSequence[0] != BallColor.UNKNOWN) {
                    break;
                }
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

    private void showLiveStats() {
        double voltage = distanceSensor.getVoltage();
        double inches = (voltage * 48.7) - 4.9;

        telemetry.addData("Dist Sensor", "Raw: %.2fV, Calc: %.1f in", voltage, inches);
        telemetry.addData("Husky ID", detectedSequence[0]);
        telemetry.update();
    }
}
