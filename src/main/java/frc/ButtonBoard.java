// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class ButtonBoard {
    private Joystick buttonBoard;
    private JoystickButton[] buttons;

    public ButtonBoard(int numberOfButtons, int buttonBoardPort) {
        buttons = new JoystickButton[numberOfButtons];
        buttonBoard = new Joystick(buttonBoardPort);
        for (int i = 1; i <= buttons.length; i++) {
            buttons[i] = new JoystickButton(buttonBoard, i);
        }
    }

    public JoystickButton getButton(int buttonIndex){
        return buttons[buttonIndex];
    }

    public Joystick getButtonBoard(){
        return buttonBoard;
    }

    public double getAxis(int axisNumber) {
        return buttonBoard.getRawAxis(axisNumber);
    }
}
