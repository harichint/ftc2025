package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Red Far Autonomous") // Changed class name
public class RedFarAutonomous extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx launcherMotor;
    private HuskyLens huskyLens;
    private NormalizedColorSensor colorSensor; // Added Color Sensor

    // --- ROBOT CONSTANTS (YOU MUST TUNE THESE!) ---
    // Drivetrain
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // Example: For a goBILDA 5203-series motor
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // Diameter of your drive wheels
    static final double ROBOT_TRACK_WIDTH_INCHES = 14.0;    // Distance between the center of left and right wheels

    // Launcher
    static final double LAUNCHER_COUNTS_PER_REV = 28;       // Ticks per rev for your launcher motor (e.g., goBILDA 5202 is 28)
    static final double LAUNCHER_GEAR_RATIO = 1.0;        // Gear ratio of the launcher (1.0 for direct drive)
    static final double LAUNCHER_WHEEL_DIAMETER_INCHES = 4.0; // Diameter of your launcher flywheel
    static final double LAUNCH_ANGLE_DEGREES = 45.0;      // Physical angle of your launcher
    static final double GRAVITY_INCHES_PER_SEC2 = 386.09;

    // --- SEQUENCE DETECTION LOGIC (from SequenceReaderOpMode) ---
    private static final int TAG_ID_1 = 1;
    private static final int TAG_ID_2 = 2;
    private static final int TAG_ID_3 = 3;

    public enum BallColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    private final BallColor[] sequenceForTag1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] sequenceForTag2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] sequenceForTag3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    private final BallColor[] sequenceUnknown = {BallColor.UNKNOWN}; // Default if no tag is found

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        initializeHardware();

        telemetry.addLine("--- Ready to Run Autonomous ---");
        telemetry.addData("Executing for Alliance", "RED");
        telemetry.addLine("\nRobot is ready!");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS EXECUTION ---

        // 1. Move forward until a white line OR an AprilTag is seen.
        telemetry.addLine("Step 1: Moving forward until white line or sequence tag is detected...");
        telemetry.update();
        BallColor[] detectedSequence = driveUntilLineOrTag(1); // Drive forward at 40% power

        // 2. Read obelisk for ball sequence, but only if we didn't already find it in Step 1.
        if (detectedSequence == null) {
            telemetry.addLine("Step 2: Reached line, now scanning for sequence...");
            telemetry.update();
            detectedSequence = readSequenceFromObelisk();
        } else {
            telemetry.addLine("Step 2: Sequence tag was found during approach!");
        }

        telemetry.addData("Sequence Found", Arrays.toString(detectedSequence));
        telemetry.update();
        sleep(1000); // Pause to read telemetry

        // Only proceed if a sequence was found
        if (detectedSequence != sequenceUnknown) {
            // 3. Turn robot right side until it finds tag ID 20 or 24
            telemetry.addLine("Step 3: Turning right until goal tag is found...");
            telemetry.update();
            turnUntilTagVisible(new int[]{20, 24}, -0.3); // Turn right (negative power) at 30% speed

            // 4. (Optional but recommended) Verify color after finding the tag
            telemetry.addLine("Verifying goal color...");
            telemetry.update();
            sleep(500); // Short pause to let robot settle

            if (isSeeingRed()) {
                telemetry.addLine("SUCCESS: Confirmed Red Goal is in view.");
            } else {
                telemetry.addLine("WARNING: Red Goal not detected by color sensor!");
                // You could add recovery logic here.
            }
            telemetry.update();
            sleep(1500); // Pause to read the result

            // 5. Goal height is 27 inches, distance is around 10 inches. Calculate projectile speed.
            telemetry.addLine("Step 5: Calculating launch velocity...");
            double goalHeight = 27.0;
            double goalDistance = 10.0;
            double requiredVelocity = calculateLaunchVelocity(goalHeight, goalDistance);

            telemetry.addData("Target Height (in)", goalHeight);
            telemetry.addData("Target Distance (in)", goalDistance);
            telemetry.addData("=> Required Velocity (ticks/s)", "%.2f", requiredVelocity);
            telemetry.update();

            // 6. Here you would spin up your launcher and shoot.

            // 7. Here come back out of launch zone by turning 45 to left and move back
            telemetry.addLine("Step 7a: Turning 45 degrees left...");
            telemetry.update();
            turnRobot(45, 0.4); // Turn 45 degrees (left)

            telemetry.addLine("Step 7b: Moving back 5 inches...");
            telemetry.update();
            driveDistance(-5, 0.4); // Move backward 5 inches at 40% power
        } else {
            // This block executes if no sequence tag was ever found.
            telemetry.addLine("ERROR: Obelisk scan failed. No sequence found.");
            telemetry.addLine("Autonomous stopping to prevent incorrect actions.");
            telemetry.update();
        }

        sleep(5000); // Wait for 5 seconds to read final telemetry
    }

    /**
     * Initializes all hardware components.
     */
    private void initializeHardware() {
        // Drivetrain
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // --- Corrected Motor Directions ---
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Launcher
        // launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        // launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Color Sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(10f); // Increase sensitivity, tune as needed

        // HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
        if (huskyLens.knock()) {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        } else {
            telemetry.addData("FATAL", "HuskyLens not responding!");
            telemetry.update();
        }
    }

    /**
     * Scans for a sequence tag (1, 2, or 3) for a short period and returns the corresponding sequence.
     * This is used for a stationary scan.
     * @return BallColor[] array representing the sequence, or UNKNOWN if not found.
     */
    private BallColor[] readSequenceFromObelisk() {
        ElapsedTime scanTimer = new ElapsedTime();
        final double SCAN_DURATION_SECONDS = 2.0; // Scan for 2 seconds

        scanTimer.reset();
        while(opModeIsActive() && scanTimer.seconds() < SCAN_DURATION_SECONDS) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Scanning... Tags Visible", blocks.length);
            telemetry.update();

            if (blocks.length > 0) {
                for (HuskyLens.Block tag : blocks) {
                    if (tag.id == TAG_ID_1) return sequenceForTag1;
                    if (tag.id == TAG_ID_2) return sequenceForTag2;
                    if (tag.id == TAG_ID_3) return sequenceForTag3;
                }
            }
            sleep(50); // Small delay to not overwhelm the hub
        }
        // If the loop finishes without finding a tag
        return sequenceUnknown;
    }

    /**
     * Checks if the color sensor is currently seeing a predominantly red color.
     * @return true if red is detected, false otherwise.
     */
    private boolean isSeeingRed() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final double RED_MULTIPLIER = 1.5;
        final float MIN_BRIGHTNESS = 0.05f;
        return (colors.alpha > MIN_BRIGHTNESS) && (colors.red > colors.blue * RED_MULTIPLIER) && (colors.red > colors.green * RED_MULTIPLIER);
    }

    /**
     * Checks if the color sensor is currently seeing a bright, white-like color.
     * @return true if white is detected, false otherwise.
     */
    private boolean isSeeingWhite() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float WHITE_BRIGHTNESS_THRESHOLD = 0.6f; // Example threshold, tune this!
        return (colors.alpha > WHITE_BRIGHTNESS_THRESHOLD);
    }

    /**
     * Drives the robot forward until the color sensor detects a white line OR a sequence AprilTag is seen.
     * @param power The motor power to use while driving (0.0 to 1.0).
     * @return The detected BallColor[] sequence if a tag is found, otherwise null.
     */
    private BallColor[] driveUntilLineOrTag(double power) {
        if (!opModeIsActive()) return null;

        setDrivePower(power);

        while (opModeIsActive()) {
            // Check for sequence tags first
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                for (HuskyLens.Block tag : blocks) {
                    if (tag.id == TAG_ID_1) {
                        setDrivePower(0);
                        return sequenceForTag1;
                    }
                    if (tag.id == TAG_ID_2) {
                        setDrivePower(0);
                        return sequenceForTag2;
                    }
                    if (tag.id == TAG_ID_3) {
                        setDrivePower(0);
                        return sequenceForTag3;
                    }
                }
            }

            // If no tag is seen, check for the white line
            if (isSeeingWhite()) {
                setDrivePower(0);
                return null; // Return null to indicate we stopped at the line, not a tag
            }

            telemetry.addData("Status", "Driving until line or tag...");
            telemetry.update();
        }

        setDrivePower(0);
        return null; // Return null if OpMode stops
    }


    /**
     * Turns the robot until one of the specified AprilTag IDs is visible.
     * @param targetIds An array of integer IDs to look for.
     * @param power The power to turn at. Positive for left, negative for right.
     */
    private void turnUntilTagVisible(int[] targetIds, double power) {
        if (!opModeIsActive()) return;

        ElapsedTime timeoutTimer = new ElapsedTime();
        final double TIMEOUT_SECONDS = 5.0; // Stop turning after 5 seconds regardless

        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);

        timeoutTimer.reset();
        boolean tagFound = false;

        while (opModeIsActive() && !tagFound && timeoutTimer.seconds() < TIMEOUT_SECONDS) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Status", "Turning to find tag...");
            telemetry.addData("Tags Visible", blocks.length);

            if (blocks.length > 0) {
                for (HuskyLens.Block tag : blocks) {
                    for (int targetId : targetIds) {
                        if (tag.id == targetId) {
                            tagFound = true;
                            telemetry.addData("SUCCESS", "Found Tag ID: " + tag.id);
                            break;
                        }
                    }
                    if (tagFound) break;
                }
            }
            telemetry.update();
            sleep(20);
        }

        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!tagFound) {
            telemetry.addLine("WARNING: Turn timed out without finding target tag.");
            telemetry.update();
        }
    }


    /**
     * Drives the robot a specific distance in inches using encoders.
     * @param distanceInches The distance to travel. Positive is forward, negative is backward.
     * @param power The motor power (0.0 to 1.0).
     */
    private void driveDistance(double distanceInches, double power) {
        if (!opModeIsActive()) return;

        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int) (distanceInches * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(targetTicks);
        rightFrontDrive.setTargetPosition(targetTicks);
        leftBackDrive.setTargetPosition(targetTicks);
        rightBackDrive.setTargetPosition(targetTicks);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            telemetry.addData("Driving", "%.2f / %.2f inches", ticksToInches(leftFrontDrive.getCurrentPosition()), distanceInches);
            telemetry.update();
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Turns the robot on the spot by a specific angle.
     * @param angle The angle in degrees. Positive is a left turn, negative is a right turn.
     * @param power The motor power (0.0 to 1.0).
     */
    private void turnRobot(double angle, double power) {
        if (!opModeIsActive()) return;

        double turnCircumference = Math.PI * ROBOT_TRACK_WIDTH_INCHES;
        double turnDistanceInches = (angle / 360.0) * turnCircumference;

        double countsPerInch = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
        int targetTicks = (int) (turnDistanceInches * countsPerInch);

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // For a left turn (positive angle), left motors go backward, right motors go forward.
        leftFrontDrive.setTargetPosition(-targetTicks);
        rightFrontDrive.setTargetPosition(targetTicks);
        leftBackDrive.setTargetPosition(-targetTicks);
        rightBackDrive.setTargetPosition(targetTicks);


        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(power);
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            telemetry.addData("Turning", "%.1f / %.1f degrees", inchesToDegrees(ticksToInches(leftFrontDrive.getCurrentPosition())), angle);
            telemetry.update();
        }
        setDrivePower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Calculates the required initial velocity to hit a target using projectile motion physics.
     * @param targetHeightY The height of the target in inches.
     * @param targetDistanceX The horizontal distance to the target in inches.
     * @return The required motor velocity in ticks per second.
     */
    private double calculateLaunchVelocity(double targetHeightY, double targetDistanceX) {
        double launchAngleRad = Math.toRadians(LAUNCH_ANGLE_DEGREES);
        double tanAngle = Math.tan(launchAngleRad);
        double cosAngle = Math.cos(launchAngleRad);

        double numerator = GRAVITY_INCHES_PER_SEC2 * Math.pow(targetDistanceX, 2);
        double denominator = 2 * Math.pow(cosAngle, 2) * (targetDistanceX * tanAngle - targetHeightY);

        if (denominator <= 0) return 0; // Target is unreachable

        double projectileVelocityInchesPerSec = Math.sqrt(numerator / denominator);
        double wheelCircumference = Math.PI * LAUNCHER_WHEEL_DIAMETER_INCHES;
        double revolutionsPerSec = projectileVelocityInchesPerSec / wheelCircumference;
        double motorRevolutionsPerSec = revolutionsPerSec * LAUNCHER_GEAR_RATIO;
        return motorRevolutionsPerSec * LAUNCHER_COUNTS_PER_REV;
    }

    // --- Helper Methods for Drivetrain ---
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

    private double ticksToInches(int ticks) {
        return (ticks / COUNTS_PER_MOTOR_REV) * (Math.PI * WHEEL_DIAMETER_INCHES);
    }

    private double inchesToDegrees(double inches) {
        return (inches / (Math.PI * ROBOT_TRACK_WIDTH_INCHES)) * 360.0;
    }
}
