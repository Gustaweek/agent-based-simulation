package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.CellBoundedPosition.{After, Before, CoordinateToCellRelation, OutOfRange, PositionExtensions, Within}
import pl.edu.agh.continuous.env.common.ToVec2Conversions.DirectionConversionExtensions
import pl.edu.agh.continuous.env.common.geometry.Vec2
import pl.edu.agh.xinuk.model.Direction
import pl.edu.agh.xinuk.model.grid.GridDirection

class CellBoundedPosition(val position: Vec2,
                          val cellSize: Double) {

  def normalize: (Vec2, Option[Direction]) =
    (Vec2(normalizeCoordinate(position.x), normalizeCoordinate(position.y)), toDirection)

  def adjust(direction: Direction, isObstacle: Boolean): Vec2 = {
    val adjustedPosition = position + direction.toVec2 * cellSize

    val cellBoundedAdjustedPosition = adjustedPosition.cellBounded(cellSize, isObstacle)
    if (cellBoundedAdjustedPosition.isOutOfRange && !isObstacle) {
      throwOutOfRangeException
    }

    adjustedPosition
  }

  def toDirection: Option[Direction] =
    coordinatesRelationToCell match {
      case (Within, Within) => None
      case (Within, After) => Some(GridDirection.Top)
      case (After, After) => Some(GridDirection.TopRight)
      case (After, Within) => Some(GridDirection.Right)
      case (After, Before) => Some(GridDirection.BottomRight)
      case (Within, Before) => Some(GridDirection.Bottom)
      case (Before, Before) => Some(GridDirection.BottomLeft)
      case (Before, Within) => Some(GridDirection.Left)
      case (Before, After) => Some(GridDirection.TopLeft)
      /*case (Within, Within) => None
      case (Within, After) => Some(GridDirection.Left)
      case (After, After) => Some(GridDirection.BottomLeft)
      case (After, Within) => Some(GridDirection.Bottom)
      case (After, Before) => Some(GridDirection.BottomRight)
      case (Within, Before) => Some(GridDirection.Right)
      case (Before, Before) => Some(GridDirection.TopRight)
      case (Before, Within) => Some(GridDirection.Top)
      case (Before, After) => Some(GridDirection.TopLeft)*/
      case _ => throwOutOfRangeException
    }

  //  case (Within, Within) => None
  //  case (Within, After) => Some(GridDirection.Top)
  //  case (After, After) => Some(GridDirection.TopRight)
  //  case (After, Within) => Some(GridDirection.Right)
  //  case (After, Before) => Some(GridDirection.BottomRight)
  //  case (Within, Before) => Some(GridDirection.Bottom)
  //  case (Before, Before) => Some(GridDirection.BottomLeft)
  //  case (Before, Within) => Some(GridDirection.Left)
  //  case (Before, After) => Some(GridDirection.TopLeft)

  def isNormalized: Boolean = position.toArray.forall(isCoordinateNormalized)

  private def isOutOfRange: Boolean = !isInRange

  private def isInRange: Boolean = position.toArray
    .forall(p => isCoordinateBeforeCell(p)
      || isCoordinateNormalized(p)
      || isCoordinateAfterCell(p))

  private def coordinatesRelationToCell: (CoordinateToCellRelation, CoordinateToCellRelation) =
    (coordinateRelationToCell(position.x), coordinateRelationToCell(position.y))

  private def coordinateRelationToCell(coordinate: Double): CoordinateToCellRelation = coordinate match {
    case p if isCoordinateNormalized(p) => Within
    case p if isCoordinateBeforeCell(p) => Before
    case p if isCoordinateAfterCell(p) => After
    case _ => OutOfRange
  }

  private def isCoordinateBeforeCell(coordinate: Double): Boolean =
    coordinate < 0.0 && coordinate >= -cellSize

  private def isCoordinateAfterCell(coordinate: Double): Boolean =
    coordinate >= cellSize && coordinate < (2.0 * cellSize)

  private def isCoordinateNormalized(coordinate: Double): Boolean =
    coordinate >= 0.0 && coordinate < cellSize

  private def normalizeCoordinate(coordinate: Double): Double = coordinate match {
    case p if isCoordinateNormalized(p) => p
    case p if isCoordinateAfterCell(p) => p - cellSize
    case p if isCoordinateBeforeCell(p) => p + cellSize
  }

  private def throwOutOfRangeException: Nothing =
    throw new IllegalStateException(s"Position $position out of neighbour cell bounds (cellSize = $cellSize)")
}

object CellBoundedPosition {
  def apply(position: Vec2, cellSize: Double, isObstacle: Boolean): CellBoundedPosition = {
    val result = new CellBoundedPosition(position, cellSize)

    if (result.isOutOfRange && !isObstacle) {
      result.throwOutOfRangeException
    }

    result
  }

  private sealed trait CoordinateToCellRelation

  private case object Before extends CoordinateToCellRelation

  private case object Within extends CoordinateToCellRelation

  private case object After extends CoordinateToCellRelation

  private case object OutOfRange extends CoordinateToCellRelation

  implicit class PositionExtensions(val position: Vec2) {
    def cellBounded(cellSize: Double, isObstacle: Boolean): CellBoundedPosition = CellBoundedPosition(position, cellSize, isObstacle)
  }
}