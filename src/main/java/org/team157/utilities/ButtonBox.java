// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The ButtonBox class is a wrapper for a {@link GenericHID} device that provides methods to access
 * button inputs. Our particular button box contains 4 switches and 6 buttons, but any amount of
 * buttons can be added or removed by modifying the {@link ButtonBoxButtons} enum.
 */
public class ButtonBox extends GenericHID {

    /** Creates a new ButtonBox */
    public ButtonBox(int port) {
        super(port);
    }

    /**
     * Ties button names to HID button numbers. Used to access a particular button's state with
     * getButton().
     */
    public static enum ButtonBoxButtons {
        /** Switch 1 - Top row, location 1 */
        SW1(1),
        /** Switch 2 - Top row, location 2 */
        SW2(2),
        /** Switch 3 - Top row, location 3 */
        SW3(3),
        /** Switch 4 - Top row, location 4 */
        SW4(4),
        /** Button 1 - Middle row, location 1 */
        BTN1(5),
        /** Button 2 - Middle row, location 2 */
        BTN2(6),
        /** Button 3 - Bottom row, location 1 */
        BTN3(7),
        /** Button 4 - Middle row, location 3 */
        BTN4(8),
        /** Button 5 - Middle row, location 4 */
        BTN5(9),
        /** Button 6 - Bottom row, location 2 */
        BTN6(10);

        private final int buttonNumber;

        ButtonBoxButtons(int buttonNumber) {
            this.buttonNumber = buttonNumber;
        }

        public int getButtonNumber() {
            return buttonNumber;
        }
    }

    /**
     * Gets the state of a particular button.
     *
     * @param button The button to get the state of, as a {@link ButtonBoxButtons} enum value
     * @return The current state of the specified button, as a {@link Trigger}
     */
    public Trigger getButton(ButtonBoxButtons button) {
        return new Trigger(() -> getRawButton(button.getButtonNumber()));
    }
}
