package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.geometry.{Line, Vec2}
import pl.edu.agh.continuous.env.model.ContinuousEnvCell
import pl.edu.agh.continuous.env.model.continuous.{Obstacle, ObstacleSegment}
import pl.edu.agh.xinuk.model.Direction

import java.util.UUID

object ObstacleMapping {
  implicit class NeighborContentsExtensions(val cells: Map[(ContinuousEnvCell, UUID), Direction]) extends AnyVal {

    def mapToObstacleLines(cellSize: Double): Iterable[Line] = {
      var lines: Iterable[Line] = Iterable.empty
      cells.foreach(cell => lines ++= mapCellToObstacleLines(cell._1._1.obstacles, Option(cell._2), cellSize))
      lines
    }
  }

  private def mapCellToObstacleLines(obstacles: Array[Obstacle], direction: Option[Direction], cellSize: Double): Iterable[Line] = {
    var lines: Iterable[Line] = Iterable.empty
    obstacles.map(obstacle => toObstacleSegments(obstacle))
      .map(segments => segments.map(segment =>
        if (direction.isEmpty) {
          Line(Vec2(segment._2.a._1, segment._2.a._2), Vec2(segment._2.b._1, segment._2.b._2))
        }
        else {
          Line(adjustObstacleVertex(segment._2.a._1, segment._2.a._2, direction.get, cellSize),
            adjustObstacleVertex(segment._2.b._1, segment._2.b._2, direction.get, cellSize))
        })
        .toIterable)
      .foreach(iterable => lines ++= iterable)
    lines
  }

  private def adjustObstacleVertex(x: Int, y: Int, direction: Direction, cellSize: Double): Vec2 = {
    val position: Vec2 = new Vec2(x, y)
    position.cellBounded(cellSize, true).adjust(direction, true)
  }

  def toObstacleSegments(obstacle: Obstacle): Array[(Int, ObstacleSegment)] = {
    var obstacleSegments: Array[(Int, ObstacleSegment)] = Array()
    for (i <- 0 until obstacle.points) {
      obstacleSegments = obstacleSegments :+ (i, ObstacleSegment((obstacle.xs(i), obstacle.ys(i)), (obstacle.xs((i + 1) % obstacle.points), obstacle.ys((i + 1) % obstacle.points))))
    }
    obstacleSegments
  }
}
