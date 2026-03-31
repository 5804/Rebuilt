package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoard {
    private Joystick buttonBoard;
    private JoystickButton[] buttons;

    public ButtonBoard(int numberOfButtons, int buttonBoardPort) {
        buttons = new JoystickButton[numberOfButtons];
        buttonBoard = new Joystick(buttonBoardPort);
        for (int i = 1; i <= buttons.length; i++) {
            buttons[i - 1] = new JoystickButton(buttonBoard, i);
        }
    }

    public JoystickButton getButton(int buttonIndex) { return buttons[buttonIndex - 1]; }

    public Joystick getButtonBoard() { return buttonBoard; }

    public JoystickButton[] getButtons() { return buttons; }
    
    public double getAxis(int axisNumber) { return buttonBoard.getRawAxis(axisNumber); }
}