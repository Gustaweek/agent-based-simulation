package pl.edu.agh.continuous.env.model

import pl.edu.agh.continuous.env.common.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.geometry.{Line, Vec2}
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.continuous.CellOutline
import pl.edu.agh.xinuk.config.{Obstacle, XinukConfig}
import pl.edu.agh.xinuk.model.continuous.{GridMultiCellId, Neighbourhood}
import pl.edu.agh.xinuk.model.{CellContents, Signal}

final case class ContinuousEnvCell(initialSignal: Signal)(implicit config: ContinuousEnvConfig) extends CellContents {
  override def generateSignal(iteration: Long)(implicit config: XinukConfig): Signal =
    initialSignal

  override def signalFactor(iteration: Long)
                           (implicit config: XinukConfig): Double = {
    ((totalCellField() - totalRunnersField) / totalCellField())
      .pow(10.0)
      .clip(lowerBound = 0.0000001, upperBound = 1.0)
  }

  private def totalCellField(): Double =
    cellOutline.width * cellOutline.height

  private def totalRunnersField: Double = runners.map(_.mass).sum

  var cellOutline: CellOutline = CellOutline.default()
  var neighbourhood: Neighbourhood = Neighbourhood.empty()
  var obstacles: Array[Obstacle] = Array()
  var runners: Array[Runner] = Array()
  var graph: Map[Vec2, Set[Vec2]] = Map.empty
  var cardinalSegments: Map[Line, GridMultiCellId] = Map.empty
  var generation: Long = 0
  var visited = false
}
