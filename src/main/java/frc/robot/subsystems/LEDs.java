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
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends AdvancedSubsystem {
    private AddressableLED strip;
    private AddressableLEDBuffer buffer;
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 120.0));
    private final LEDPattern greenPattern = LEDPattern.solid(Color.kGreen)
        .breathe(Seconds.of(5));
    private LEDPattern bluePattern = LEDPattern.solid(Color.kBlue);
    //private LEDPattern orangePattern = LEDPattern.solid(Color.kOrange);
    private final LEDPattern orangePattern = LEDPattern.solid(Color.kOrange);
    // TODO Change 0.5 to IMU tilt
    private final LEDPattern imuPattern = LEDPattern.progressMaskLayer(() -> 0.5)
        .mask(LEDPattern.solid(Color.kBlue));
    private LEDPattern activePattern;
    private AnimationTypes currentAnimation;

    public enum AnimationTypes {
        GreenBreeze,
        BlueTilt,
        OrangeSolid,
        Rainbow
    }

    public LEDs() {
        strip = new AddressableLED(Constants.LEDs.stripPwm);
        buffer = new AddressableLEDBuffer(Constants.LEDs.stripLength);

        strip.setLength(buffer.getLength());
        strip.setData(buffer);
        strip.start();

        activePattern = rainbow;
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none();
    }

    public void changeAnimation(AnimationTypes anim) {
        if(currentAnimation == anim) return;
        currentAnimation = anim;
        switch(anim) {
            case GreenBreeze:
                activePattern = greenPattern;
                break;
            case BlueTilt:
                activePattern = imuPattern;
                break;
            case OrangeSolid:
                activePattern = orangePattern;
                break;
            default:
            case Rainbow:
                activePattern = rainbow;
                break;
        }
    }
    
    @Override
    public void periodic() {
        // Has Coral, Green
        if(RobotContainer.coralHandler.hasCoral()) changeAnimation(AnimationTypes.GreenBreeze);
        activePattern.applyTo(buffer);
        strip.setData(buffer);
    }
}
