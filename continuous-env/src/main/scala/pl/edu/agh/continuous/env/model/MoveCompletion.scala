package pl.edu.agh.continuous.env.model

import pl.edu.agh.continuous.env.common.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.geometry.Vec2
import pl.edu.agh.continuous.env.model.MoveCompletion.{roundedValue, safeValue}


final case class MoveCompletion(value: Double, normal: Option[Vec2], penalty: Boolean, tag: String) extends Ordered[MoveCompletion] {
  override def compare(that: MoveCompletion): Int = value compareTo that.value

  def safeRounded: MoveCompletion = MoveCompletion(roundedValue(safeValue(value)), normal, penalty, tag)

  def withNormal(normal: Vec2): MoveCompletion = MoveCompletion(value, normal, tag)

  def withoutPenalty: MoveCompletion = MoveCompletion(value, normal, penalty = false, tag)

  def withTagPrefix(prefix: String): MoveCompletion = MoveCompletion(value, normal, penalty, s"${prefix}_$tag")

  override def canEqual(that: Any): Boolean = that.isInstanceOf[MoveCompletion]

  override def equals(other: Any): Boolean = other match {
    case that@MoveCompletion(thatValue, _, _, _) =>
      (that canEqual this) &&
        value == thatValue
    case _ => false
  }

  override def hashCode(): Int = {
    val state = Seq(value)
    state.map(_.hashCode()).foldLeft(0)((a, b) => 31 * a + b)
  }
}

case object MoveCompletion {
  private val maxValue = 1.0
  private val minValue = 0.0
  private val delta = 0.0001

  def apply(value: Double, tag: String): MoveCompletion =
    if (isValueWithinBounds(value)) {
      new MoveCompletion(clippedValue(value), None, penalty = true, tag)
    } else {
      throw InvalidMoveCompletionValueException(value)
    }

  def apply(value: Double, normal: Vec2, tag: String): MoveCompletion =
    if (isValueWithinBounds(value)) {
      if (isNormalNonZero(normal)) {
        new MoveCompletion(clippedValue(value), Some(normal), penalty = true, tag)
      } else {
        throw InvalidMoveCompletionNormalException(normal)
      }
    } else {
      throw InvalidMoveCompletionValueException(value)
    }

  def tryApply(value: Double, tag: String): Option[MoveCompletion] =
    if (isValueWithinBounds(value)) {
      Some(MoveCompletion(value, tag))
    } else {
      None
    }

  def tryApply(value: Double, normal: Vec2, tag: String): Option[MoveCompletion] =
    if (isValueWithinBounds(value) && isNormalNonZero(normal)) {
      Some(MoveCompletion(value, normal, tag))
    } else {
      None
    }

  def min(tag: String = ""): MoveCompletion = MoveCompletion(minValue, tag)

  def max(tag: String = ""): MoveCompletion = MoveCompletion(maxValue, tag)

  def isValueWithinBounds(value: Double): Boolean = value >= (0.0 - delta) && value <= (100.0 + delta) //TODO must be cell size + delta

  private def isNormalNonZero(normal: Vec2): Boolean = normal.lengthSq != 0.0

  private def clippedValue(value: Double): Double = value.clip(lowerBound = minValue, upperBound = maxValue)

  private def safeValue(value: Double): Double = Seq(value - 0.01, minValue).max

  private def roundedValue(value: Double): Double = if (value < 0.1) minValue else value

  abstract class InvalidMoveCompletionException(message: String) extends Exception(message)

  case class InvalidMoveCompletionValueException(value: Double)
    extends InvalidMoveCompletionException(s"Invalid move completion value: $value")

  case class InvalidMoveCompletionNormalException(normal: Vec2)
    extends InvalidMoveCompletionException(s"Invalid move completion normal: $normal")

  implicit class IterableOfMoveCompletionExtensions(val it: Iterable[MoveCompletion]) extends AnyVal {
    def minByValueOption: Option[MoveCompletion] = it.minByOption(_.value)

    def minByValue: MoveCompletion = it.minBy(_.value)
  }
}