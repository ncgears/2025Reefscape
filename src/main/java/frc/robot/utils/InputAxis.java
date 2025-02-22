package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;

// ### slew rate limiter
// ### https://github.com/frc6995/Robot-2024/blob/main/src/main/java/frc/robot/util/drive/AsymmetricSlewRateLimiter.java

/** An extension of DoubleSupplier designed for pre-processing driver controller axis inputs. */
public class InputAxis implements DoubleSupplier {
  DoubleSupplier m_supplier;
  double deadband = 0;
  SlewRateLimiter limiter = new SlewRateLimiter(100,-100,0);
  boolean square;
  double multiplier = 1;

  double outputValue;
  String name;

  public InputAxis(String name, DoubleSupplier supplier) {
    this.name = name;
    m_supplier = supplier;
  }

  // @Override
  // public String configureLogName() {
  //     return name;
  // }

  // private double inputValue() {
  //   return m_supplier.getAsDouble();
  // }

  public InputAxis withSlewRate(double forward, double back) {
    limiter = new SlewRateLimiter(forward, back,0);
    return this;
  }

  public InputAxis withSlewRate(double rate) {
    limiter = new SlewRateLimiter(rate,-rate,0);
    return this;
  }

  public InputAxis withMultiplier(double multiplier) {
    this.multiplier = multiplier;
    return this;
  }

  public InputAxis withDeadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  public InputAxis withSquaring(boolean square) {
    this.square = square;
    return this;
  }

  public InputAxis withInvert(boolean invert) {
    this.multiplier = Math.abs(this.multiplier) * (invert ? -1 : 1);
    return this;
  }

  public void resetSlewRate() {
    double value = m_supplier.getAsDouble();
    value *= multiplier;
    value = MathUtil.applyDeadband(value, deadband);
    if (this.square) {
      value = Math.copySign(value * value, value);
    }
    limiter.reset(value);
  }

  @Override
  public double getAsDouble() {
    double value = m_supplier.getAsDouble();
    value *= multiplier;
    value = MathUtil.applyDeadband(value, deadband);
    if (this.square) {
      value = Math.copySign(value * value, value);
    }
    value = limiter.calculate(value);
    outputValue = value;
    return value;
  }
}