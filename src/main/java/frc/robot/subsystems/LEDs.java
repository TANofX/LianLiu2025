/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The CANdle MultiAnimation example demonstrates using multiple animations with CANdle.
 * This example has the robot using a Command Based template to control the CANdle.
 * 
 * This example uses:
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animations.
 * 
 * Controls (with Xbox controller):
 * Right Bumper: Increment animation
 * Left Bumper: Decrement animation
 * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
 * POV Right: Configure maximum brightness for the CANdle
 * POV Down: Configure medium brightness for the CANdle
 * POV Left: Configure brightness to 0 for the CANdle
 * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
 * A: Print the VBat voltage in Volts
 * B: Print the 5V voltage in Volts
 * X: Print the current in amps
 * Y: Print the temperature in degrees C
 * 
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

/**
 * The LEDs subsystem controls an addressable LED strip using various patterns
 * and animations.
 */
public class LEDs extends AdvancedSubsystem {
    private final AlgaeHandler algaeHandler;
    private final CoralHandler coralHandler;
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDs.LENGTH);
    private static final Distance LED_SPACING = Meters.of(1.0/60);
    private static final LinearVelocity LED_VELOCITY = InchesPerSecond.of(2);
     // Set patterns for use
     private final LEDPattern rainbow = LEDPattern.rainbow(255, 255)
             .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 120.0));

     private final LEDPattern greenPattern = LEDPattern.solid(Color.kGreen);

     private final LEDPattern whitePattern = LEDPattern.solid(Color.kBlue);

     // Intake wave
     private final LEDPattern greenWave = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kGreen).scrollAtAbsoluteSpeed(LED_VELOCITY.times(1), LED_SPACING);
     private final LEDPattern whiteWave = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, Color.kBlue).scrollAtAbsoluteSpeed(LED_VELOCITY.times(1), LED_SPACING);

     private final LEDPattern standby = LEDPattern.solid(Color.kRed);

     //private final LEDPattern assisted = LEDPattern.blink(Seconds.of(1.5, 5.0));

     // Outake wave
     //private final LEDPattern greenOWave = LEDPattern.solid(Color.kGreen).scrollAtAbsoluteSpeed(LED_VELOCITY.times(-1), LED_SPACING);
     private final LEDPattern greenOWave = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kGreen).scrollAtAbsoluteSpeed(LED_VELOCITY.times(-1), LED_SPACING);
     private final LEDPattern whiteOWave = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, Color.kWhite).scrollAtAbsoluteSpeed(LED_VELOCITY.times(-1), LED_SPACING);

     //Segments to not define in periodic
     private final AddressableLEDBufferView coralSegmentA = buffer.createView(22, 44);

     private final AddressableLEDBufferView coralSegmentB = buffer.createView(72, 51);

     private final AddressableLEDBufferView algaeSegmentA = buffer.createView(0,22);

     private final AddressableLEDBufferView algaeSegmentB = buffer.createView(72,94);

     private final AddressableLEDBufferView full = buffer.createView(0, 94);
    /**
     * Constructs an LEDs subsystem and initializes the LED strip and buffer.5
     */
    public LEDs(AlgaeHandler algaeHandler, CoralHandler coralHandler) {
        this.algaeHandler = algaeHandler;
        this.coralHandler = coralHandler;
        strip = new AddressableLED(Constants.LEDs.PWM_PIN);

        strip.setLength(buffer.getLength());
        strip.setData(buffer);
        strip.start();
    }

    /**
     * This class does not have to perform any checks, so this method returns a
     * command that does nothing.
     *
     * @return A command that does nothing.
     */
    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }

    /**
     * Periodically updates the LED strip with the active pattern.
     */
    @Override
    public void periodic() {

         // Signals for coral
         if (coralHandler.hasCoral()) {
            whitePattern.applyTo(coralSegmentA);
            whitePattern.applyTo(coralSegmentB);
            // System.out.println("LED acknowledge collected Coral");
        } else if (coralHandler.getIntaking() == 1) {
            whiteWave.applyTo(coralSegmentA);
            whiteWave.applyTo(coralSegmentB);
            // System.out.println("LED acknowledge intaking Coral");
        } else if (coralHandler.getIntaking() == -1) {
            whiteOWave.applyTo(coralSegmentA);
            whiteOWave.applyTo(coralSegmentB);
            // System.out.println("LED acknowledge outaking Coral");
        } else {
            standby.applyTo(coralSegmentA);
            standby.applyTo(coralSegmentB);
        }

         // Signals for aglae
       if (algaeHandler.hasAlgae()) {
           greenPattern.applyTo(algaeSegmentA);
           greenPattern.applyTo(algaeSegmentB);
        //    System.out.println("LED acknowledge collected Algae");
       }else if (algaeHandler.getIntaking() == 1){
           greenWave.applyTo(algaeSegmentA);
           greenWave.applyTo(algaeSegmentB);
        //    System.out.println("LED acknowledge intaking Algae");
       } else if (algaeHandler.getIntaking() == -1){
               greenOWave.applyTo(algaeSegmentA);
               greenOWave.applyTo(algaeSegmentB);
            //    System.out.println("LED acknowledge outaking Algae");
       } else {
           standby.applyTo(algaeSegmentA);
           standby.applyTo(algaeSegmentB);

       }

       // Left and right ID lights, happen to be same colors as other signals

       standby.applyTo(buffer.createView(44, 47));
       greenPattern.applyTo(buffer.createView(48, 51));
        /*
         * if(timeForLED & usingAuto){
         * turn on leds
         * }
         */

        strip.setData(buffer);
    }
}
