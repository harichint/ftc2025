package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.Arrays;

@Autonomous(name = "Red Front Autonomous")
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

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        initializeHardware();

        // Use the pre-run SequenceReaderOpMode to get the randomization
        SequenceReaderOpMode.BallColor[] detectedSequence = SequenceReaderOpMode.result.sequence;

        telemetry.addLine("--- Ready to Run Autonomous ---");
        telemetry.addData("Executing for Alliance", "RED");
        telemetry.addData("Using Detected Sequence", Arrays.toString(detectedSequence));
        telemetry.addLine("\nRobot is ready!");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS EXECUTION ---

        // 1. From far white triangle move to center white triangle.
        telemetry.addLine("Step 1: Moving 70 inches to center...");
        telemetry.update();
        driveDistance(70, 0.5); // Drive 70 inches at 50% power
        //Verify if on white line with color sensor
        telemetry.addLine("Verifying white line...");
        telemetry.update();
        sleep(500); // Short pause for sensor reading

        if (isSeeingWhite()) {
            telemetry.addLine("SUCCESS: Confirmed robot is on the white line.");
        } else {
            telemetry.addLine("WARNING: White line not detected!");
        }
        telemetry.update();
        sleep(1500); // Pause to read the result

        // 2. Read obelisk for ball sequence, print on telemetry
        // The sequence is already read from SequenceReaderOpMode. We just display it again.
        telemetry.addLine("Step 2: Displaying pre-detected sequence...");
        telemetry.addData("Sequence", Arrays.toString(detectedSequence));
        telemetry.update();
        sleep(1000); // Pause to read telemetry

        // 3. Turn robot right side 45 degrees and confirm the goal is Red Goal.
        telemetry.addLine("Step 3: Turning 45 degrees right...");
        telemetry.update();
        turnRobot(-45, 0.4); // Turn -45 degrees (right) at 40% power

        // In a real scenario, you might re-scan with HuskyLens here to confirm the red goal.
        // For this example, we will assume the turn is correct.
        //verify with color sensor if the red goal is in view..
        telemetry.addLine("Verifying goal color...");
        telemetry.update();
        sleep(500); // Short pause to let robot settle

        if (isSeeingRed()) {
            telemetry.addLine("SUCCESS: Confirmed Red Goal is in view.");
        } else {
            telemetry.addLine("WARNING: Red Goal not detected by color sensor!");
            // You could add recovery logic here, like turning a bit more or rescanning.
        }
        telemetry.update();
        sleep(1500); // Pause to read the result

        // 4. Goal height is 27 inches, distance is around 10 inches. Calculate projectile speed.
        telemetry.addLine("Step 4: Calculating launch velocity...");
        double goalHeight = 27.0;
        double goalDistance = 10.0;
        double requiredVelocity = calculateLaunchVelocity(goalHeight, goalDistance);

        telemetry.addData("Target Height (in)", goalHeight);
        telemetry.addData("Target Distance (in)", goalDistance);
        telemetry.addData("=> Required Velocity (ticks/s)", "%.2f", requiredVelocity);
        telemetry.update();

        // 5. Here you would spin up your launcher to the 'requiredVelocity'
        // launcherMotor.setVelocity(requiredVelocity);
        // And then shoot the projectile.
        // 6. Here come back out of launch zone by turning 45 to left and move back
        telemetry.addLine("Step 6a: Turning 45 degrees left...");
        telemetry.update();
        turnRobot(45, 0.4); // Turn 45 degrees (left)

        telemetry.addLine("Step 6b: Moving back 5 inches...");
        telemetry.update();
        driveDistance(-5, 0.4); // Move backward 5 inches at 40% power

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

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Launcher enable when launching is added in the build
//        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor"); // Example name
//        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Color Sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color"); // Add to config
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
     * Checks if the color sensor is currently seeing a predominantly red color.
     * @return true if red is detected, false otherwise.
     * use Husky lens camera to find the 20 / 24 tag to identify the goal
     */
    private boolean isSeeingRed() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // This logic checks if the red component is significantly stronger than blue and green.
        // The multiplier makes it more robust than just checking for the highest value.
        // You may need to tune the multiplier and the minimum brightness threshold.
        final double RED_MULTIPLIER = 1.5;
        final float MIN_BRIGHTNESS = 0.05f;

        // Return true if brightness is sufficient AND red is dominant
        return (colors.alpha > MIN_BRIGHTNESS) && (colors.red > colors.blue * RED_MULTIPLIER) && (colors.red > colors.green * RED_MULTIPLIER);
    }

    /**
     * Checks if the color sensor is currently seeing a bright, white-like color.
     * @return true if white is detected, false otherwise.
     */
    private boolean isSeeingWhite() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // For white, all three color components (R, G, B) should be high and relatively balanced.
        // We check if the overall brightness (alpha) is above a certain threshold.
        // You MUST tune this threshold by pointing the sensor at the white line and observing the alpha value.
        final float WHITE_BRIGHTNESS_THRESHOLD = 0.6f; // Example threshold, tune this!

        return (colors.alpha > WHITE_BRIGHTNESS_THRESHOLD);
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

        leftFrontDrive.setTargetPosition(targetTicks);
        rightFrontDrive.setTargetPosition(-targetTicks);
        leftBackDrive.setTargetPosition(targetTicks);
        rightBackDrive.setTargetPosition(-targetTicks);

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
