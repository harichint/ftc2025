package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;

/*
 * This OpMode uses a HuskyLens camera to detect an Alliance Tag (4 or 5)
 * and a randomization Tag (1, 2, or 3), storing the results for a separate
 * Autonomous OpMode to use.
 */
@TeleOp(name = "HuskyLens Sequence Reader", group = "Sensor")
public class SequenceReaderOpMode extends LinearOpMode {

    // --- Sensor Object ---
    private HuskyLens huskyLens;

    // --- Define the Target AprilTag IDs ---
    private static final int TAG_ID_1 = 1;
    private static final int TAG_ID_2 = 2;
    private static final int TAG_ID_3 = 3;
    private static final int RED_TAG_ID_4 = 4;
    private static final int BLUE_TAG_ID_5 = 5;

    // --- Define the Colors and Color Sequences ---
    public enum BallColor {
        GREEN,
        PURPLE,
        RED,
        BLUE,
        UNKNOWN // Added for default cases
    }

    private final BallColor[] sequenceForTag1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] sequenceForTag2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] sequenceForTag3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    // ================== DATA CONTAINER CLASS ==================
    /**
     * A static class to hold the detected alliance and color sequence.
     * This makes the data clean and easy to access from another OpMode.
     */
    public static class DetectionResult {
        public BallColor alliance = BallColor.UNKNOWN; // Default value
        public BallColor[] sequence = {}; // Default to empty array
    }
    // ===========================================================

    // Create a single, static instance of our results object that will persist.
    public static DetectionResult result = new DetectionResult();

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        try {
            huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");

            if (!huskyLens.knock()) {
                telemetry.addData("FATAL", "HuskyLens found but not responding!");
                telemetry.update();
                while (!isStopRequested()) { sleep(50); }
                return;
            }

            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

            telemetry.addData(">>", "HuskyLens Initialized.");
            telemetry.addData(">>", "Looking for Alliance (4,5) and Sequence (1,2,3) Tags.");
            telemetry.update();

        } catch (IllegalArgumentException e) {
            telemetry.addData("FATAL", "Hardware Device 'huskylens' not found. Check configuration.");
            telemetry.update();
            while (!isStopRequested()) { sleep(50); }
            return;
        }

        waitForStart();

        // --- CONTINUOUS SCANNING LOOP ---
        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.clearAll();
            telemetry.addData("Total Tags Visible", blocks.length);

            // Loop through all visible tags and update our results object
            if (blocks.length > 0) {
                for (HuskyLens.Block tag : blocks) {
                    processTag(tag.id);
                }
            }

            // Display the currently stored results
            printCurrentResults();
            telemetry.update();
            sleep(100);
        }
    }

    /**
     * Checks a tag ID and updates the static 'result' object accordingly.
     * @param parsedID The ID of the detected AprilTag.
     */
    private void processTag(int parsedID) {
        // Check for Alliance Tags
        if (parsedID == RED_TAG_ID_4) {
            result.alliance = BallColor.RED;
        } else if (parsedID == BLUE_TAG_ID_5) {
            result.alliance = BallColor.BLUE;
        }
        // Check for Sequence Tags
        else if (parsedID == TAG_ID_1) {
            result.sequence = sequenceForTag1;
        } else if (parsedID == TAG_ID_2) {
            result.sequence = sequenceForTag2;
        } else if (parsedID == TAG_ID_3) {
            result.sequence = sequenceForTag3;
        }
    }

    /**
     * Helper method to display the current state of the DetectionResult object.
     */
    private void printCurrentResults() {
        telemetry.addLine("--- CURRENT AUTONOMOUS PLAN ---");
        telemetry.addData("Detected Alliance", result.alliance);
        telemetry.addData("Detected Sequence", Arrays.toString(result.sequence));
        telemetry.addLine("-------------------------------");
        telemetry.addLine("OpMode can be stopped when results are correct.");
    }
}
