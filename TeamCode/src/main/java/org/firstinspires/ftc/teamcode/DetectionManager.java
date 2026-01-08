//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.SequenceReaderOpMode.BallColor.GREEN;
//import static org.firstinspires.ftc.teamcode.SequenceReaderOpMode.BallColor.PURPLE;
//
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import android.graphics.Color;
//
///**
// * A manager class to handle all sensor readings for autonomous setup.
// * It contains the logic from SequenceReaderOpMode and BallColorRecognition.
// */
//public class DetectionManager {
//
//    // --- Sensor Objects ---
//    private HuskyLens huskyLens;
//    private NormalizedColorSensor colorSensor;
//
//    // --- Result Storage ---
//    public enum Alliance { RED, BLUE, UNKNOWN }
////    public String [] ballColor ={String.valueOf(GREEN), String.valueOf(PURPLE), String.valueOf(PURPLE)};
//
//    public static class DetectionResult {
//        public Alliance alliance = Alliance.UNKNOWN;
//        public SequenceReaderOpMode.BallColor[] sequence = {};
//        public SequenceReaderOpMode.BallColor pixelColor = SequenceReaderOpMode.BallColor.UNKNOWN;
//    }
//    public DetectionResult result = new DetectionResult();
//
//    // --- HuskyLens Constants ---
//    private static final int TAG_ID_1 = 1, TAG_ID_2 = 2, TAG_ID_3 = 3;
//    private static final int RED_TAG_ID_4 = 4, BLUE_TAG_ID_5 = 5;
//    private final SequenceReaderOpMode.BallColor[] SEQ_1 = {SequenceReaderOpMode.BallColor.GREEN, SequenceReaderOpMode.BallColor.PURPLE, SequenceReaderOpMode.BallColor.PURPLE};
//    private final SequenceReaderOpMode.BallColor[] SEQ_2 = {SequenceReaderOpMode.BallColor.PURPLE, SequenceReaderOpMode.BallColor.GREEN, SequenceReaderOpMode.BallColor.PURPLE};
//    private final SequenceReaderOpMode.BallColor[] SEQ_3 = {SequenceReaderOpMode.BallColor.PURPLE, SequenceReaderOpMode.BallColor.PURPLE, SequenceReaderOpMode.BallColor.GREEN};
//
//    /**
//     * Constructor: Initializes all sensors.
//     * @param hardwareMap The hardware map from the OpMode.
//     */
//    public DetectionManager(HardwareMap hardwareMap) {
//        // Initialize HuskyLens
//        huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
//        if (huskyLens.knock()) {
//            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//        }
//
//        // Initialize Color Sensor
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        colorSensor.setGain(16.0f);
//    }
//
//    /**
//     * Reads visible AprilTags and updates the result object.
//     */
//    public void detectFromHuskyLens() {
//        HuskyLens.Block[] blocks = huskyLens.blocks();
//        if (blocks.length == 0) return;
//
//        for (HuskyLens.Block tag : blocks) {
//            if (tag.id == RED_TAG_ID_4) result.alliance = Alliance.RED;
//            else if (tag.id == BLUE_TAG_ID_5) result.alliance = Alliance.BLUE;
//            else if (tag.id == TAG_ID_1) result.sequence = SEQ_1;
//            else if (tag.id == TAG_ID_2) result.sequence = SEQ_2;
//            else if (tag.id == TAG_ID_3) result.sequence = SEQ_3;
//        }
//    }
//
//    /**
//     * Reads the color sensor and updates the result object.
//     */
//    public void detectFromColorSensor() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        float[] hsvValues = new float[3];
//        Color.colorToHSV(colors.toColor(), hsvValues);
//        float hue = hsvValues[0];
//
//        if (hue >= 90 && hue <= 167) {
//            result.pixelColor = SequenceReaderOpMode.BallColor.PURPLE;
//        } else if (hue >= 200 && hue <= 280) {
//            result.pixelColor = SequenceReaderOpMode.BallColor.GREEN;
//        } else {
//            result.pixelColor =  SequenceReaderOpMode.BallColor.UNKNOWN;
//        }
//    }
//}
