package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Commands.ElevatorCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/* OI instantiates the joysticks and controllers used by drivers to operate the robot
      The buttons on these controllers are also instantiated in order to run commands when they are pressed */
public class OI {

  //declaring joysticks to be used
  public static Joystick
  leftStick, // 
  rightStick,
  controller, // 
  auxStick; // 

//declaring buttons for each joystick
public static JoystickButton 
  triggerRight, // 
  triggerLeft, //
  triggerAux, // 

  button2Right, // 
  button3Right, // 
  button4Right, // 
  button5Right, //
  button6Right, //
  button7Right, // 
  button8Right, // 
  button9Right, // 
  button10Right, // 
  button11Right, //
  button12Right, //

  button2Aux, // 
  button3Aux, // 
  button4Aux, // 
  button5Aux, // 
  button6Aux, // 
  button7Aux, // 
  button8Aux, // 
  button9Aux, // 
  button10Aux, // 
  button11Aux, // 
  button12Aux, //

  button2Left, // 
  button3Left, // 
  button4Left, // 
  button5Left, // 
  button6Left, // 
  button7Left, // 
  button8Left, // 
  button9Left, // 
  button10Left, // 
  button11Left, //
  button12Left,
  
  buttonA,
  buttonB,
  buttonY,
  buttonX,
  buttonLB,
  buttonRB,
  buttonBox,
  buttonPancake;// 

  //POV buttons for logitech joystick, with directional input
  public static POVButton 
    povButtonUp,
    povButtonDown;
  
  //constructor which runs once, and defines all joysticks and buttons
  public OI() {

    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    auxStick = new Joystick(2);
    //XboxController controller = new XboxController(3);

    //controller is an Xbox controller created using the Joystick class
    //can alternatively use the XboxController class (probably works better)
    controller = new Joystick(3);

    triggerLeft = new JoystickButton(leftStick, 1);
    triggerRight = new JoystickButton(rightStick, 1);
    triggerAux = new JoystickButton(auxStick, 1);

    button2Right = new JoystickButton(rightStick, 2);  
    button3Right = new JoystickButton(rightStick, 3); 
    button4Right = new JoystickButton(rightStick, 4); 
    button5Right = new JoystickButton(rightStick, 5); 
    button6Right = new JoystickButton(rightStick, 6); 
    button7Right = new JoystickButton(rightStick, 7); 
    button8Right = new JoystickButton(rightStick, 8); 
    button9Right = new JoystickButton(rightStick, 9); 
    button10Right = new JoystickButton(rightStick, 10); 
    button11Right = new JoystickButton(rightStick, 11); 
    button12Right = new JoystickButton(rightStick, 12); 

    button2Aux = new JoystickButton(auxStick, 2);  
    button3Aux = new JoystickButton(auxStick, 3); 
    button4Aux = new JoystickButton(auxStick, 4); 
    button5Aux = new JoystickButton(auxStick, 5); 
    button6Aux = new JoystickButton(auxStick, 6); 
    button7Aux = new JoystickButton(auxStick, 7); 
    button8Aux = new JoystickButton(auxStick, 8); 
    button9Aux = new JoystickButton(auxStick, 9); 
    button10Aux = new JoystickButton(auxStick, 10); 
    button11Aux = new JoystickButton(auxStick, 11); 
    button12Aux = new JoystickButton(auxStick, 12); 


    povButtonUp = new POVButton(auxStick, 0);
    povButtonDown = new POVButton(auxStick, 180);

    button2Left = new JoystickButton(leftStick, 2);
    button3Left = new JoystickButton(leftStick, 3); 
    button4Left = new JoystickButton(leftStick, 4); 
    button5Left = new JoystickButton(leftStick, 5); 
    button6Left = new JoystickButton(leftStick, 6); 
    button7Left = new JoystickButton(leftStick, 7); 
    button8Left = new JoystickButton(leftStick, 8); 
    button9Left = new JoystickButton(leftStick, 9); 
    button10Left = new JoystickButton(leftStick, 10); 
    button11Left = new JoystickButton(leftStick, 11); 
    button12Left = new JoystickButton(leftStick, 12);

    //buttons for Xbox controller
    buttonA = new JoystickButton(controller, 1);  
    buttonB = new JoystickButton(controller, 2);
    buttonX = new JoystickButton(controller, 3); 
    buttonY = new JoystickButton(controller, 4); 
    buttonLB = new JoystickButton(controller, 5); 
    buttonRB = new JoystickButton(controller, 6); 
    buttonBox = new JoystickButton(controller, 7); 
    buttonPancake = new JoystickButton(controller, 8); 
    
  }
}