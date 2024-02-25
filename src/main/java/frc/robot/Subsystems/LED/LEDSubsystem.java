// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static final CANdle m_candle = new CANdle(Constants.CanConstants.LED_CANDLE);

  /* Colors */

  public static final Color black = new Color(0, 0, 0);         // This will Turn off the CANdle
  public static final Color green = new Color(50, 168, 82);     // Green
  public static final Color red = new Color(171, 41, 43);       // Red
  public static final Color yellow = new Color(107,107,199);    // Yellow
  public static final Color white = new Color(255, 230, 220);   // White
  public static final Color blue = new Color(8, 32, 255);       // Blue
  public static final Color purple = new Color(184, 0, 185);    // Purple

  /* Animations */

  //public static final Animation ColorFlow = new ColorFlowAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex)
  public static final Animation m_ready2Shoot = new StrobeAnimation(green.red, green.green, green.blue, 0, 0.09, 300, 0);  // Flash Green
  public static final Animation m_notReady2Shoot = new StrobeAnimation(red.red, red.green, red.blue, 0, .09, 300, 0);  // Flash Red




  public LEDSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(candleConfiguration, 100);

    setDefaultCommand(defaultCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command defaultCommand() {
    return runOnce(() -> {
        LEDSegment.MainStrip.setColor(yellow);
        LEDSegment.SecondStrip.setColor(yellow);
    });
  }

  public void setBrightness(double percent) {
    /* Here we will set the brightness of the LEDs */
    m_candle.configBrightnessScalar(percent, 100);
  }

  /* This section will define our LED strip and apply the proper Functions to change it */
  public static enum LEDSegment {
        MainStrip(8, 300, 2),
        SecondStrip(0,100,3);

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            m_candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            m_candle.animate(animation, animationSlot);
        }

        public void ready2Shoot(int slotNum) {
            m_candle.animate(m_ready2Shoot, slotNum);
        }

        public void notReady2Sheet(int slotNum) {
            m_candle.animate(m_notReady2Shoot, slotNum);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            m_candle.clearAnimation(0);
        }

        public void disableLEDs() {
            setColor(black);
        }

        /*public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }
        */
    }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
  }
}
