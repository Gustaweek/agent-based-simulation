package pl.edu.agh.continuous.env.model

import pl.edu.agh.continuous.env.common.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model.{CellContents, Signal}

final case class RunnerOccupied(generation: Long, runners: Set[Runner]) extends CellContents {
  override def generateSignal(iteration: Long)
                             (implicit config: XinukConfig): Signal = Signal.zero
  //    config.asInstanceOf[GeoKinConfig].singleRunnerInitialSignal * runners.map(_.mass).sum

  override def signalFactor(iteration: Long)
                           (implicit config: XinukConfig): Double = {
    ((totalCellField - totalRunnersField) / totalCellField)
      .pow(10.0)
      .clip(lowerBound = 0.0000001, upperBound = 1.0)
  }

  private def totalCellField(implicit config: XinukConfig): Double =
    math.pow(config.asInstanceOf[ContinuousEnvConfig].cellSize, 2.0)

  private def totalRunnersField: Double = runners.map(_.mass).sum
}