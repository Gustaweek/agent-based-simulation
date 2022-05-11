package pl.edu.agh.continuous.env.model

import io.jvm.uuid.UUID
import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.geometry.{Circle, SweptCircle, Vec2}
import pl.edu.agh.xinuk.model.Direction

import java.awt.Color
import scala.util.Random

final case class Runner(id: UUID,
                        generation: Long,
                        priority: Long,
                        position: Vec2,
                        radius: Double,
                        nextStep: Vec2,
                        var path: List[Vec2],
                        speed: Double,
                        lastMoveCompletion: Option[MoveCompletion],
                        isStuck: Boolean,
                        color: Color) extends Equals {

  def body: Circle = Circle(position, radius)

  def mass: Double = body.area

  def sweptBody: SweptCircle = body.sweep(nextStep)

  def sweptBody(moveCompletion: MoveCompletion): SweptCircle = body.sweep(nextStep * moveCompletion.value)

  def endPosition(moveCompletion: MoveCompletion): Vec2 = position + (nextStep * moveCompletion.value)

  def endPosition: Vec2 = position + nextStep

  def maxStepLength(cellSize: Double): Double = cellSize * 0.5 - body.r //this was diameter but should be radius

  def lastActualStep: Option[Vec2] = lastMoveCompletion.map(lmc => nextStep * lmc.value)

  def completeMove(moveCompletion: MoveCompletion): Runner = Runner(
    id,
    generation,
    priority,
    position + (nextStep * moveCompletion.value),
    radius,
    nextStep,
    path,
    speed,
    Some(moveCompletion),
    moveCompletion.tag.equals("e"),
    color
  )

  def withNewPriority(): Runner = Runner(
    id,
    generation,
    Random.nextLong(),
    position,
    radius,
    nextStep,
    path,
    speed,
    lastMoveCompletion,
    isStuck,
    color
  )

  def withNextStep(nextStep: Vec2): Runner = Runner(id, generation, priority, position, radius, nextStep, path, speed, None, isStuck, color)

  def withAdjustedPosition(cellSize: Double,
                           direction: Direction): Runner = {
    val newPosition = position.cellBounded(cellSize, false).adjust(direction, false)
    Runner(id, generation, priority, newPosition, radius, nextStep, path, speed, lastMoveCompletion, isStuck, color)
  }

  def withIncrementedGeneration(): Runner = Runner(
    id,
    generation + 1,
    priority,
    position,
    radius,
    nextStep,
    path,
    speed,
    lastMoveCompletion,
    isStuck,
    color
  )

  def normalizePosition(cellSize: Double): (Runner, Option[Direction]) = {
    val (newPosition, maybeDirection) = position.cellBounded(cellSize, false).normalize
    (Runner(id, generation, priority, newPosition, radius, nextStep, path, speed, lastMoveCompletion, isStuck, color), maybeDirection)
  }

  def inflate(radiusDelta: Double): Runner = Runner(
    id,
    generation,
    priority,
    position,
    radius + radiusDelta,
    nextStep,
    path,
    speed,
    lastMoveCompletion,
    isStuck,
    color
  )

  override def canEqual(that: Any): Boolean = that.isInstanceOf[Runner]

  override def equals(other: Any): Boolean = other match {
    case that: Runner =>
      (that canEqual this) &&
        id == that.id
    case _ => false
  }

  override def hashCode(): Int = {
    val state = Seq(id)
    state.map(_.hashCode()).foldLeft(0)((a, b) => 31 * a + b)
  }
}

object Runner {
  def apply(id: UUID,
            generation: Long,
            priority: Long,
            position: Vec2,
            radius: Double,
            nextStep: Vec2,
            path: List[Vec2],
            speed: Double,
            lastMoveCompletion: Option[MoveCompletion],
            isStuck: Boolean,
            color: Color): Runner =
    new Runner(
      id,
      generation,
      priority,
      position,
      radius,
      nextStep,
      path,
      speed,
      lastMoveCompletion,
      isStuck = isStuck,
      color)

  def createNew(position: Vec2,
                radius: Double,
                nextStep: Vec2,
                speed: Double,
                color: Color): Runner =
    new Runner(
      UUID.random,
      generation = 0,
      priority = Random.nextLong(),
      position,
      radius,
      nextStep,
      List.empty,
      speed,
      None,
      isStuck = false,
      color)

  def createNew(start: Vec2,
                end: Vec2,
                radius: Double,
                speed: Double,
                color: Color): Runner =
    new Runner(
      UUID.random,
      generation = 0,
      priority = Random.nextLong(),
      start,
      radius,
      end - start,
      List.empty,
      speed,
      None,
      isStuck = false,
      color)

  def createNew(position: Vec2,
                radius: Double,
                speed: Double,
                color: Color): Runner =
    new Runner(
      UUID.random,
      generation = 0,
      priority = Random.nextLong(),
      position,
      radius,
      nextStep = Vec2.zero,
      List.empty,
      speed,
      None,
      isStuck = false,
      color)

  def createNewMock(sweptCircle: SweptCircle,
                    speed: Double,
                    color: Color): Runner =
    new Runner(
      UUID(0, 0),
      generation = 0,
      priority = 0,
      position = sweptCircle.start,
      radius = sweptCircle.r,
      nextStep = sweptCircle.line.vector,
      List.empty,
      speed,
      None,
      isStuck = false,
      color)
}