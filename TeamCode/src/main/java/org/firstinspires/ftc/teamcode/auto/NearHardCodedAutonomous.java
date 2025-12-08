package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Near Hardcoded Auto")

public class NearHardCodedAutonomous extends LinearOpMode {

        // --- HARDWARE ---
        private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
        private HuskyLens huskyLens;
        private NormalizedColorSensor colorSensor;
        private CRServo intakeGate;
        private DcMotor conveyorBelt;

        // --- ROBOT CONSTANTS (Tune These!) ---
        static final double COUNTS_PER_MOTOR_REV = 537.7;    // For goBILDA 5203-series motor
        static final double WHEEL_DIAMETER_INCHES = 3.78;
        static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;   // Distance between left and right wheels
        static final double LAUNCH_ANGLE_DEGREES = 40.0;
        static final double OBELISK_SCAN_ANGLE_DEGREES = 45.0;
        private static final double SHOOTER_CONVEYOR_POWER = 1.0;
        private static final double GATE_SERVO_POWER = 1.0;

        // --- ALLIANCE SELECTION ---
        private enum GoalDirection { LEFT, RIGHT }
        private GoalDirection selectedAlliance = GoalDirection.RIGHT; // Default to Right side goal

        // --- SEQUENCE LOGIC ---
        public enum BallColor { GREEN, PURPLE, UNKNOWN }
        private final BallColor[] SEQ_1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
        private final BallColor[] SEQ_2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
        private final BallColor[] SEQ_3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
        private final BallColor[] SEQ_UNKNOWN = {BallColor.UNKNOWN};

        @Override
        public void runOpMode() {
            initializeHardware();

            // =============================== ALLIANCE SELECTION LOOP ===============================
            // This loop runs during the INIT phase, before the driver presses START.
            while (!isStarted() && !isStopRequested()) {
                telemetry.addLine("--- Alliance Selection ---");
                telemetry.addData("Selected Alliance", selectedAlliance);
                telemetry.addLine("\nPress 'B' on Gamepad 1 for Right side goal post");
                telemetry.addLine("Press 'X' on Gamepad 1 for Left side goal post");
                telemetry.update();

                if (gamepad1.b) {
                    selectedAlliance = GoalDirection.RIGHT;
                }
                if (gamepad1.x) {
                    selectedAlliance = GoalDirection.LEFT;
                }
            }
            // =====================================================================================

            telemetry.addData("Ready to Run for", selectedAlliance + " Alliance");
            telemetry.update();

            waitForStart();

            // --- AUTONOMOUS SEQUENCE ---

            // Step 1: Drive diagonally - 68 inches backward.
            telemetry.addLine("Step 1: Driving 34 inches backward.");
            telemetry.update();
            driveMecanum(-68, 35, 0.7); // Drive 70 forward, 35 right, at 70% power
            // Step 2: Turn to read sequnce based on selectedAlliance
            telemetry.addLine("Step 2: Turning to read sequence.");
            telemetry.update();
            if (selectedAlliance == GoalDirection.RIGHT) {
                turnRobot(OBELISK_SCAN_ANGLE_DEGREES, 0.5); // Turn Left for Right side Alliance
            } else { // Alliance is BLUE
                turnRobot(-OBELISK_SCAN_ANGLE_DEGREES, 0.5);  // Turn Right for LEFT side Alliance
            }
            // Step 3: Now that we have arrived, scan for the sequence tag obelisk.
            telemetry.addLine("Step 3: Arrived, now scanning obelisk...");
            telemetry.update();
            BallColor[] detectedSequence = readSequenceFromObelisk(2.0); // Scan for 2 seconds

            telemetry.addData("Sequence Found", Arrays.toString(detectedSequence));
            telemetry.update();
            sleep(1000);

            // --- Conditional Logic: Only proceed if a sequence was found ---
            if (detectedSequence[0] != BallColor.UNKNOWN) {

                // Step 3: Turn towards the correct goal based on the selected alliance and the same angle as the steps 2.
                telemetry.addLine("Step 3: Turning towards goal...");
                telemetry.update();
                if (selectedAlliance == GoalDirection.RIGHT) {
                    turnRobot(-OBELISK_SCAN_ANGLE_DEGREES, 0.5); // Turn RIGHT for Right side Alliance
                } else { // Alliance example is BLUE
                    turnRobot(OBELISK_SCAN_ANGLE_DEGREES, 0.5);  // Turn LEFT for LEFT side Alliance
                }

                // Step 4: shooting.
                runShooter();
                telemetry.addLine("Step 4: Shoot 3 balls...");
                telemetry.update();
                sleep(1500); // Run shooter for 1.5 seconds

                // Step 5: Drive backward to come out of launch line.
                telemetry.addLine("Step 6: Driving backward...");
                telemetry.update();
                driveDistance(-30, 0.4);  // Move backward
            } else {
                // If the obelisk scan failed, stop here.
                telemetry.addLine("ERROR: No sequence found. Stopping.");
                telemetry.update();
            }

            sleep(3000); // End of OpMode
        }

        /**
         * Activates the shooter-specific components (conveyor and gate servo).
         */
        public void runShooter() {
            conveyorBelt.setPower(SHOOTER_CONVEYOR_POWER);
            intakeGate.setPower(GATE_SERVO_POWER);
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

            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            intakeGate = hardwareMap.get(CRServo.class, "intake_gate");
            conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor_belt");

            // --- Set Motor & Servo Directions ---
            intakeGate.setDirection(DcMotorSimple.Direction.FORWARD);   // Spins to help feed conveyor
            conveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD); // Spins to push balls up/out

            // Sensors
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            colorSensor.setGain(10f);

            huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
            if (!huskyLens.knock()) {
                telemetry.addData("FATAL", "HuskyLens not responding!");
            }
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        /** Performs a stationary scan for a sequence tag for a given duration. */
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
                sleep(50);
            }
            telemetry.addLine("ERROR: No sequence found. Default Sequence loaded");
            return SEQ_1;
        }

        /** Drives the robot a specific distance in inches using encoders. */
        private void driveDistance(double distanceInches, double power) {
            if (!opModeIsActive()) return;
            driveMecanum(distanceInches, 0, power); // Use the new method for straight movement
        }

        /** Drives the robot diagonally using backward and strafe distances. */
        private void driveMecanum(double backwardInches, double strafeInches, double power) {
            if (!opModeIsActive()) return;

            double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

            // Mecanum drive formulas to calculate individual wheel targets
            int lfTicks = (int)((backwardInches + strafeInches) * countsPerInch);
            int rfTicks = (int)((backwardInches - strafeInches) * countsPerInch);
            int lbTicks = (int)((backwardInches - strafeInches) * countsPerInch);
            int rbTicks = (int)((backwardInches + strafeInches) * countsPerInch);

            setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setTargetPosition(lfTicks, rfTicks, lbTicks, rbTicks);
            setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            setDrivePower(power);
            while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
                // Optional: telemetry while driving
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
