// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.HubTracker;
// import frc.robot.RobotContainer;
// import frc.robot.HubTracker.Shift;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.util.Color;

// import java.util.stream.IntStream;

// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.configs.*;
// import com.ctre.phoenix6.signals.*;
// import com.ctre.phoenix6.controls.*;

// import frc.robot.Constants.LEDConstants;
 
// public class LED extends SubsystemBase {
//     final CANdle candleLeft;
//     final CANdle candleRight;
//     final int NUM_TOTAL_LEDS;
    
//     final Alliance ALLIANCE;
//     // Used for testing, cycled manually
//     public static Shift currentSimulatedHubShift;
//     public Shift currentHubShift;
//     public static Alliance simulatedAutoWinner;
    
//     public LED() {
//         candleLeft = new CANdle(LEDConstants.LEFT_CANDLE_ID);
//         candleRight = new CANdle(LEDConstants.RIGHT_CANDLE_ID);

//         CANdleConfiguration config = new CANdleConfiguration();
//         config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
//         config.LED.StripType = StripTypeValue.GRB;
//         config.LED.BrightnessScalar = 0.1;
//         config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;

//         candleLeft.getConfigurator().apply(config);
//         candleRight.getConfigurator().apply(config);

//         NUM_TOTAL_LEDS = LEDConstants.CANDLE_NUM_LEDS + LEDConstants.STRIP_NUM_LEDS; // CANdle + LED strip
//         ALLIANCE = RobotContainer.alliance.get();
//         currentSimulatedHubShift = Shift.AUTO;
//         currentHubShift = LEDConstants.IS_TESTING ? currentSimulatedHubShift : HubTracker.getCurrentShift().get();
//         simulatedAutoWinner = Alliance.Blue;
//         // isClimbing = false;
//     }

//     // ----------------------------------------------------------
//     // ---------- Normal color and animation methods ------------
//     // ----------------------------------------------------------

//     public void setColor(Color color) {
//         // candleLeft.clearAllAnimations();
//         // candleRight.clearAllAnimations();

//         SolidColor colorRequest = new SolidColor(0, NUM_TOTAL_LEDS-1);
//         colorRequest = colorRequest.withColor(ColorRGBW.getColor(color));

//         candleLeft.setControl(colorRequest);
//         candleRight.setControl(colorRequest);
//     }
//     public void setStrobeAnimation(Color color, double frequency) {
//         StrobeAnimation animationRequest = new StrobeAnimation(0, NUM_TOTAL_LEDS-1);
//         animationRequest = animationRequest.withColor(ColorRGBW.getColor(color));
//         animationRequest = animationRequest.withFrameRate(frequency);
        
//         candleLeft.setControl(animationRequest);
//         candleRight.setControl(animationRequest);
//     }
//     public void setFireAnimation() {
//         FireAnimation animationRequest = new FireAnimation(0, NUM_TOTAL_LEDS-1);
//         animationRequest = animationRequest.withCooling(0.2);
//         animationRequest = animationRequest.withSparking(0.3);
//         animationRequest = animationRequest.withFrameRate(27);
        
//         candleLeft.setControl(animationRequest);
//         candleRight.setControl(animationRequest);
//     }
//     public void setRainbowAnimation() {
//         RainbowAnimation animationRequest = new RainbowAnimation(0, NUM_TOTAL_LEDS-1);

//         candleLeft.setControl(animationRequest);
//         candleRight.setControl(animationRequest);
//     }

//     public void off() {
//         setColor(Color.kBlack);
//         setStrobeAnimation(Color.kBlack, 0);
//     }

//     // ------------------------------
//     // ---------- Climbing ----------
//     // ------------------------------

//     // public int getNumLitLEDs() {
//     //     double heading = DriveSubsystem.getHeading();
//     //     int headingModulus = (int) Math.round(Math.abs(heading)) % 360;
//     //     double pitch = DriveSubsystem.getPitch();
//     //     final double LED_SCALAR = NUM_LEDS / 180.0; // Scale the 180 degrees down to however many LEDs we have

//     //     if (headingModulus < 160) {
//     //         return (int) Math.round(Math.abs(pitch*LED_SCALAR));
//     //     } else if (headingModulus >= 160) {
//     //         return (int) Math.round(NUM_LEDS - (Math.abs(pitch*LED_SCALAR)));
//     //     } else {
//     //         return 0;
//     //     }
//     // }

//     // public SolidColor getRainbowLedRequest(int frame) {
//     //     int startLED = 1;
//     //     int endLED = getNumLitLEDs();
//     //     SolidColor request = new SolidColor(startLED-1, endLED-1);

//     //     int r;
//     //     int g;
//     //     int b;

//     //     // Total of 1530 frames for rainbow animation (255*6)
//     //     // 255,0,0 - 255,255,0 - 0,255,0 - 0,255,255 - 0,0,255 - 255,0,255
//     //     if (frame >= 0 && frame <= 255*1) { // 255, 0^, 0;
//     //         r = 255;
//     //         g = frame;
//     //         b = 0;
//     //     } else if (frame > 255 && frame <= 255*2) { // 255v, 255, 0
//     //         r = 255*2 - frame;
//     //         g = 255;
//     //         b = 0;
//     //     } else if (frame > 255*2 && frame < 255*3) { // 0, 255, 0^
//     //         r = 0;
//     //         g = 255;
//     //         b = frame - 255*2;
//     //     } else if (frame > 255*3 && frame <= 255*4) { // 0, 255v, 255
//     //         r = 0;
//     //         g = 255*4 - frame;
//     //         b = 255;
//     //     } else if (frame > 255*4 && frame <= 255*5) { // 0^, 0, 255
//     //         r = frame - 255*4;
//     //         g = 0;
//     //         b = 255;
//     //     } else if (frame > 255*5 && frame <= 255*6) { // 255, 0, 255v
//     //         r = 255;
//     //         g = 0;
//     //         b = 255*6 - frame;
//     //     } else {
//     //         r = 255;
//     //         g = 255;
//     //         b = 255;
//     //     }

//     //     request = request.withColor(new RGBWColor(r, g, b));
//     //     return request;
//     // }
//     // public SolidColor getBlinkLedRequest(int frame) {
//     //     int startLED = getNumLitLEDs();
//     //     int endLED = NUM_LEDS;
//     //     SolidColor request = new SolidColor(startLED-1, endLED-1);

//     //     int framesPerBlink = (int) Math.round(50/endgameBlinkFrequencyGlobal);

//     //     // f blinks/second
//     //     // 50 frames/second
//     //     // 50/f frames/blink

//     //     IntStream onFrames = IntStream.range(1, framesPerBlink/2+1);
        
//     //     if (onFrames.anyMatch(n -> n == frame)) {
//     //         // System.out.println("ON");
//     //         request = request.withColor(new RGBWColor(Color.kWhite)); // On
//     //     } else {
//     //         // System.out.println("OFF");
//     //         request = request.withColor(new RGBWColor(Color.kBlack)); // Off
//     //     }
        
//     //     return request;
//     // }

//     // public void setClimb(int rainbowFrame, int blinkFrame) {
//     //     candleLeft.clearAllAnimations();

//     //     SolidColor requestRainbow = getRainbowLedRequest(rainbowFrame);
//     //     SolidColor requestBlink = getBlinkLedRequest(blinkFrame);

//     //     candleLeft.setControl(requestRainbow);
//     //     candleLeft.setControl(requestBlink);
//     // }

//     // ----------------------------------
//     // ---------- Hub Shifting ----------
//     // ----------------------------------

//     public void solidAlliance(Alliance alliance) {
//         if (alliance == ALLIANCE) {
//             setColor(Color.kLime);
//         } else {
//             setColor(Color.kRed);
//         }
//     }
//     public void blinkAlliance(Alliance alliance) {
//         if (alliance == ALLIANCE) {
//             setStrobeAnimation(Color.kLime, LEDConstants.NORMAL_BLINKING_FREQUENCY);
//         } else {
//             setStrobeAnimation(Color.kRed, LEDConstants.NORMAL_BLINKING_FREQUENCY);
//         }
//     }

//     public void solidBoth() {
//         setColor(Color.kYellow);
//     } 
//     public void blinkBoth() {
//         setStrobeAnimation(Color.kYellow, LEDConstants.NORMAL_BLINKING_FREQUENCY);
//     }

//     public void blinkEndgame(double frequency) {
//         setStrobeAnimation(Color.kWhite, frequency);
//     }

//     public void rainbow() {
//         setRainbowAnimation();
//     }
//     public void fire() {
//         setFireAnimation();
//     }

//     Alliance autoWinner = null;
//     Alliance autoLoser = null;
//     public void changeLED(HubTracker.Shift hubShift) {
//         switch (hubShift) {
//             case AUTO -> {
//                 isClimbing = false;
//                 solidBoth();
//             }

//             case TRANSITION -> {
//                 // Only update the auto winner and loser when it is actually necessary
//                 autoWinner = LEDConstants.IS_TESTING ? simulatedAutoWinner : HubTracker.getAutoWinner().get(); 
//                 autoLoser = (autoWinner == Alliance.Red) ? Alliance.Blue : Alliance.Red;
//                 solidBoth();
//             }
//             case TRANSITION_BLINK -> blinkAlliance(autoLoser);

//             case SHIFT_1 -> solidAlliance(autoLoser); // Auto loser hub active
//             case SHIFT_1_BLINK -> blinkAlliance(autoWinner);

//             case SHIFT_2 -> solidAlliance(autoWinner); // Auto winner hub active
//             case SHIFT_2_BLINK -> blinkAlliance(autoLoser);

//             case SHIFT_3 -> solidAlliance(autoLoser);
//             case SHIFT_3_BLINK -> blinkAlliance(autoWinner);

//             case SHIFT_4 -> solidAlliance(autoWinner);
//             case SHIFT_4_BLINK -> blinkBoth();

//             case ENDGAME -> solidBoth(); // Blinking yellow faster as endgame progresses
//             case ENDGAME_BLINK_1 -> {
//                 // isClimbing = true;
//                 // endgameBlinkFrequencyGlobal = 0.5;
//                 blinkEndgame(1);
//             }
//             case ENDGAME_BLINK_2 -> {
//                 // isClimbing = true;
//                 // endgameBlinkFrequencyGlobal = 1;
//                 blinkEndgame(2);
//             }
//             case ENDGAME_BLINK_3 -> {
//                 // isClimbing = true; 
//                 // endgameBlinkFrequencyGlobal = 2;
//                 blinkEndgame(4);
//             }
//         }
//     }

//     int shiftIndex = 0;
//     Shift[] shifts = Shift.values();
//     public void cycleHubShift() {
//         if (shiftIndex < 14) {
//             shiftIndex++;
//         } else {
//             shiftIndex = 0;
//         }
//         currentSimulatedHubShift = shifts[shiftIndex];
//         System.out.println(currentSimulatedHubShift);
//     }

//     int rainbowFrameGlobal = 1;
//     int blinkFrameGlobal = 1;
//     double endgameBlinkFrequencyGlobal = 0;
//     public static boolean isClimbing = false;
//     public void periodic() { 
//         // if (isClimbing) {
//         //     if (rainbowFrameGlobal >= 1530) {
//         //         rainbowFrameGlobal = 1;
//         //     } else {
//         //         rainbowFrameGlobal += 4;
//         //     }
//         //     if (blinkFrameGlobal >= 50/endgameBlinkFrequencyGlobal) {
//         //         blinkFrameGlobal = 1;
//         //     } else {
//         //         blinkFrameGlobal++;
//         //     }

//             // setClimb(rainbowFrameGlobal, blinkFrameGlobal);

//         Shift initialHubShift = LEDConstants.IS_TESTING ? currentSimulatedHubShift : HubTracker.getCurrentShift().get();
    
//         if (currentHubShift != initialHubShift) { // Only changes LED when the shift changes
//             currentHubShift = initialHubShift;
//             System.out.println(currentHubShift);
//             changeLED(currentHubShift); // This line is buggy as of Match 80 at Districts. 
//         }
//     }

    
//     public static class ColorRGBW extends Color {
//         public ColorRGBW() {
//             super();
//         }

//         public static RGBWColor getColor(Color c) {
//             return new RGBWColor((int) Math.round(c.red*255), (int) Math.round(c.green*255), (int) Math.round(c.blue*255));
//         }
//     }
// }
