package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class ComputerVisionSub {
  private SerialPort m_arduinoPort;
  private double pixy_xVal = -1, pixy_SetPointVal = -1;

  public ComputerVisionSub() {
        try {
          m_arduinoPort = new SerialPort(115200, SerialPort.Port.kUSB);
          System.out.println("Connected to arduino in vision!");
        } catch (Exception e) {
          System.out.println("kUSB failed");
    
          try {
            m_arduinoPort = new SerialPort(115200, SerialPort.Port.kUSB1);
            System.out.println("Connected to arduino in vision!");
          } catch (Exception e1) {
            System.out.println("kUSB1 failed");
            
            try {
              m_arduinoPort = new SerialPort(115200, SerialPort.Port.kUSB2);
              System.out.println("Connected to arduino in vision!"); 
            } catch(Exception e2){
              System.out.println("kUSB2 failed");
            }
          }
        }
  }

  public void readData() {
    try {
      pixy_xVal = Integer.parseInt(m_arduinoPort.readString().substring(0, m_arduinoPort.readString().indexOf("|")));
      pixy_SetPointVal = Integer.parseInt(m_arduinoPort.readString().substring(m_arduinoPort.readString().indexOf("|") + 1));
    } catch(Exception e) {
      pixy_xVal = -1;
      pixy_SetPointVal = -1;
    }
  }

  public double getPixyXValue() {
    return pixy_xVal;
  }

  public double getPixySetPointValue() {
    return pixy_SetPointVal;
  }

  public double getError() {
    return pixy_xVal - pixy_SetPointVal;
  }
}