package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.MathConstants

object MathUtils {
  implicit class DoubleExtensions(val value: Double) {
    def clip(lowerBound: Double, upperBound: Double): Double = value.clipLower(lowerBound).clipUpper(upperBound)

    def clipUpper(upperBound: Double): Double = Math.min(value, upperBound)

    def clipLower(lowerBound: Double): Double = Math.max(value, lowerBound)

    def ~=(otherValue: Double): Boolean = value.equalsWithDelta(otherValue, MathConstants.epsilon)

    def equalsWithDelta(otherValue: Double, epsilon: Double): Boolean = (value - otherValue).abs < epsilon

    def pow(power: Double): Double = math.pow(value, power)
  }
}
