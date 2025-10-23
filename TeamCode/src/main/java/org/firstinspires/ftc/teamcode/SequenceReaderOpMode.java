package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;

/*
 * This OpMode uses a HuskyLens camera to intake AprilTags and display a
 * predefined color sequence based on the detected tag's ID.
 */
@TeleOp(name = "HuskyLens Color Sequence Display", group = "Sensor")
public class SequenceReaderOpMode extends LinearOpMode {

    // --- Sensor Object ---
    private HuskyLens huskyLens;

    // --- Define the Target AprilTag IDs from your request ---
    private static final int TAG_ID_1 = 1;
    private static final int TAG_ID_2 = 2;
    private static final int TAG_ID_3 = 3;

    private static final int RED_TAG_ID_4 = 4;
    private static final int BLUE_TAG_ID_5 = 5;


    // --- Define the Colors and the Color Sequences from your request ---
    public enum BallColor {
        GREEN,
        PURPLE,
        RED,
        BLUE
    }

    // Associate each pattern with a variable based on your specifications.
    private final BallColor[] sequenceForTag1 = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    private final BallColor[] sequenceForTag2 = {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    private final BallColor[] sequenceForTag3 = {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        try {
            // Initialize the HuskyLens from the hardware map
            huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");

            // Check if the HuskyLens is communicating
            if (!huskyLens.knock()) {
                telemetry.addData("FATAL", "HuskyLens found but not responding!");
                telemetry.update();
                while (!isStopRequested()) { sleep(50); }
                return;
            }

            // Set the algorithm to AprilTag Recognition
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

            telemetry.addData(">>", "HuskyLens Initialized.");
            telemetry.addData(">>", "Looking for Tags 1, 2, or 3.");
            telemetry.addData(">>", "Press Start to begin scanning.");
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
            // Get all detected AprilTags from the HuskyLens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.clearAll();

            telemetry.addData("Total Tags Detected", blocks.length);
            telemetry.addLine("---");

            if (blocks.length > 0) {
                boolean targetTagFound = false;
                // Loop through every tag the HuskyLens has detected
                for (HuskyLens.Block tag : blocks) {
                    int parsedID = tag.id;

                    // Check if the parsed ID matches one of our targets
                    if (parsedID == TAG_ID_1) {
                        printSequenceForTag(parsedID, sequenceForTag1);
                        targetTagFound = true;
                        break; // Stop after finding the first valid tag
                    } else if (parsedID == TAG_ID_2) {
                        printSequenceForTag(parsedID, sequenceForTag2);
                        targetTagFound = true;
                        break;
                    } else if (parsedID == TAG_ID_3) {
                        printSequenceForTag(parsedID, sequenceForTag3);
                        targetTagFound = true;
                        break;
                    }
                }
                if (!targetTagFound) {
                    telemetry.addLine("No target tags (1, 2, 3) are visible.");
                }
            } else {
                telemetry.addLine("No tags visible.");
            }

            telemetry.update();
            sleep(100);
        }
    }

    /**
     * Helper method to print the details of a detected tag and its associated color sequence.
     * @param tagId The ID of the detected AprilTag.
     * @param sequence The color sequence associated with that tag ID.
     */
    private void printSequenceForTag(int tagId, BallColor[] sequence) {
        telemetry.addLine(String.format("▼▼▼ DETECTED TAG ID: %d ▼▼▼", tagId));
        telemetry.addData("  -> Color Sequence", Arrays.toString(sequence));
        telemetry.addLine("▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲");
    }
}
