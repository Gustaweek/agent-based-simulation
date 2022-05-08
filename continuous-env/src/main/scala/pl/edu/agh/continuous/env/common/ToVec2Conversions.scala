package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.geometry.Vec2
import pl.edu.agh.xinuk.model.SignalMap.signalMap2Map
import pl.edu.agh.xinuk.model.grid.GridDirection
import pl.edu.agh.xinuk.model.{Direction, SignalMap}

import scala.language.implicitConversions

object ToVec2Conversions {

  implicit class DirectionConversionExtensions(val direction: Direction) {
    def toVec2: Vec2 = direction match {
      case GridDirection.Top => Vec2(0, 1)
      case GridDirection.TopRight => Vec2(1, 1)
      case GridDirection.Right => Vec2(1, 0)
      case GridDirection.BottomRight => Vec2(1, -1)
      case GridDirection.Bottom => Vec2(0, -1)
      case GridDirection.BottomLeft => Vec2(-1, -1)
      case GridDirection.Left => Vec2(-1, 0)
      case GridDirection.TopLeft => Vec2(-1, 1)
      /*case GridDirection.Top => Vec2(-1, 0)
      case GridDirection.TopRight => Vec2(-1, -1)
      case GridDirection.Right => Vec2(0, -1)
      case GridDirection.BottomRight => Vec2(1, -1)
      case GridDirection.Bottom => Vec2(1, 0)
      case GridDirection.BottomLeft => Vec2(1, 1)
      case GridDirection.Left => Vec2(0, 1)
      case GridDirection.TopLeft => Vec2(-1, 1)*/
      case _ => throw new UnsupportedOperationException("Unknown direction")
    }
  }

  //  case GridDirection.Top => Vec2(0, 1)
  //  case GridDirection.TopRight => Vec2(1, 1)
  //  case GridDirection.Right => Vec2(1, 0)
  //  case GridDirection.BottomRight => Vec2(1, -1)
  //  case GridDirection.Bottom => Vec2(0, -1)
  //  case GridDirection.BottomLeft => Vec2(-1, -1)
  //  case GridDirection.Left => Vec2(-1, 0)
  //  case GridDirection.TopLeft => Vec2(-1, 1)

  implicit class SignalMapConversionExtensions(val signalMap: SignalMap) {
    def toVec2: Vec2 = signalMap2Map(signalMap)
      .map { case (direction, signal) => (direction.toVec2.normalized, signal) }
      .map { case (vec, signal) => vec * signal.value }
      .fold(Vec2.zero)((v1: Vec2, v2: Vec2) => v1 + v2)
  }

}
