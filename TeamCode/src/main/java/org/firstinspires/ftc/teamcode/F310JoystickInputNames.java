package org.firstinspires.ftc.teamcode;

/**
 * This is NOT  an opmode
 *
 * This enum is used to define the names of all inputs in a Logitech F310 Gamepads to allow for
 * flexibilities like allowing the driver to select an input method and the programmer to be more
 * literal in what is being programmed. NOTE this does not replace class Gamepad and its methods. It
 * merely gives names to inputs on a gamepad.
 *
 * Version History
 * ======= =======
 * v 0.1   11/2/18 Added this Javadoc comment
 *
 */

public enum F310JoystickInputNames {
    DPAD; //Only one DPAD. Enum unnecessary.
    enum Joysticks{
        LEFT_STICK, RIGHT_STICK
    }

    enum ABXYButtons{
        A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON
    }

    enum Bumpers{
        LEFT_BUMPER, RIGHT_BUMPER
    }

    enum Triggers{
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

}
