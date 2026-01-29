package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Far Left Auto")
public class FarLeftAutonomous extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private HuskyLens huskyLens;
    private NormalizedColorSensor colorSensor;
    private AnalogInput distanceSensor;

    private DcMotor intakeRoller;
    private CRServo rightIntakeGate;
    private DcMotor rightConveyorBelt; // Shooter
    private CRServo leftIntakeGate;
    private DcMotor leftConveyorBelt; // Shooter

    // --- ROBOT CONSTANTS ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double ROBOT_TRACK_WIDTH_INCHES = 22.0;   
    static final double LAUNCH_ANGLE_DEGREES = 60.0;
    private static final double SHOOTER_CONVEYOR_POWER = 1.0;
    private static final double GATE_SERVO_POWER = 1.0;

    // --- ALLIANCE & SEQUENCE ---
    private enum GoalDirection { LEFT, RIGHT }
    private GoalDirection selectedAlliance = GoalDirection.RIGHT;
    
    public enum BallColor { GREEN, PURPLE, UNKNOWN }
    private BallColor[] detectedSequence = {BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
    
    private final BallColor[] SEQ_3 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] SEQ_1 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    @Override
    public void runOpMode() {
        initializeHardware();

        selectedAlliance = GoalDirection.LEFT;
        boolean selectionMade = true;

        waitForStart();

        if (opModeIsActive() && selectionMade) {
            telemetry.addData("Status", "Scanning while moving...");
            telemetry.update();

            // Step 1: Drive forward and scan. 
            // It will stop as soon as it sees a tag or hits 60 inches.
            double distanceToTag = driveMecanum(60, 0, 0.4, true); 
            
            // Show found sequence and how far we went
            telemetry.addData("Final Sequence", Arrays.toString(detectedSequence));
            telemetry.addData("Distance to Tag", "%.2f in", distanceToTag);
            telemetry.update();
            
            if (detectedSequence[0] == BallColor.UNKNOWN) {
                telemetry.addLine("Scanning one last time at stop...");
                telemetry.update();
                detectedSequence = readSequenceFromObelisk(3.0);
            }

            sleep(500);

            // Step 3: Return the EXACT distance traveled
            driveMecanum(-distanceToTag, 0, 0.7, false);

            if (detectedSequence[0] != BallColor.UNKNOWN) {
                // Step 4: Aim
                if (selectedAlliance == GoalDirection.RIGHT) {
                    turnRobot(-40, 0.5); 
                } else {
                    turnRobot(40, 0.5); 
                }

                runShootingSequence(detectedSequence);

                if (selectedAlliance == GoalDirection.RIGHT) {
                    turnRobot(40, 0.5);
                } else {
                    turnRobot(-40, 0.5);
                }

                driveDistance(12, 0.5, false);

                double sideTurnAngle = (selectedAlliance == GoalDirection.RIGHT) ? -90 : 90;
                turnRobot(sideTurnAngle, 0.5);

                // Step 8: Move and Intake
                intakeRoller.setPower(0.8);
                leftIntakeGate.setPower(0.8);
                leftConveyorBelt.setPower(-0.8);
                rightConveyorBelt.setPower(-0.8);

                driveDistance(36, 0.3, false);
                driveDistance(6, 0.1, false);      
                sleep(200);
                stopAllMechanisms();

                driveDistance(-42, 0.6, false);
                turnRobot(-sideTurnAngle, 0.5); 
                driveDistance(-12, 0.5, false);

                if (selectedAlliance == GoalDirection.RIGHT) {
                    turnRobot(-40, 0.5);
                } else {
                    turnRobot(40, 0.5);
                }

                runShootingSequence(detectedSequence);
                showLiveStats();
            } else {
                telemetry.addLine("ERROR: No sequence found. Stopping.");
                telemetry.update();
            }
        }
    }

    public void runShootingSequence(BallColor[] sequence) {
        for (BallColor color : sequence) {
            if (color == BallColor.GREEN) {
                leftConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
                sleep(500); 
                leftIntakeGate.setPower(GATE_SERVO_POWER); 
                intakeRoller.setPower(0.6); 
                sleep(600); 
                stopLeftShooter();
            } else if (color == BallColor.PURPLE) {
                rightConveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
                sleep(500); 
                rightIntakeGate.setPower(GATE_SERVO_POWER);
                intakeRoller.setPower(0.6);
                sleep(600);
                stopRightShooter();
            }
            intakeRoller.setPower(0);
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
        rightConveyorBelt = hardwareMap.get(DcMotor.class, "right_conveyor_belt");
        leftIntakeGate = hardwareMap.get(CRServo.class, "left_intake_gate");
        leftConveyorBelt = hardwareMap.get(DcMotor.class, "left_conveyor_belt");

        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeGate.setDirection(DcMotorSimple.Direction.REVERSE);
        rightConveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeGate.setDirection(DcMotorSimple.Direction.FORWARD);
        leftConveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
        
        distanceSensor = hardwareMap.get(AnalogInput.class, "ranger");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(10f);

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
        BallColor[] unknownSeq = {BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
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
            sleep(50); 
        }
        return unknownSeq;
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
            if (scan && detectedSequence[0] == BallColor.UNKNOWN) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                if (blocks.length > 0) {
                    for (HuskyLens.Block tag : blocks) {
                        if (tag.id == 1) detectedSequence = SEQ_1;
                        else if (tag.id == 2) detectedSequence = SEQ_2;
                        else if (tag.id == 3) detectedSequence = SEQ_3;
                    }
                }
                
                // If sequence found, stop immediately
                if (detectedSequence[0] != BallColor.UNKNOWN) {
                    break;
                }
            }
            showLiveStats();
            sleep(50); 
        }

        // Return actual distance traveled
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
        double sensorVoltage = distanceSensor.getVoltage();
        double distanceInches = (sensorVoltage * 48.7) - 4.9;
        telemetry.addData("Sequence", Arrays.toString(detectedSequence));
        telemetry.addData("Dist", "%.1f in", distanceInches);
        telemetry.update();
    }
}
