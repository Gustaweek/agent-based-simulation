package pl.edu.agh.continuous.env

import com.typesafe.scalalogging.LazyLogging
import pl.edu.agh.continuous.env.algorithm.{ContinuousEnvMetrics, ContinuousEnvWorldCreator, GeoKinMetrics, GeoKinPlanCreator, GeoKinPlanResolver}
import pl.edu.agh.continuous.env.model.ContinuousEnvCell
import pl.edu.agh.xinuk.Simulation
import pl.edu.agh.xinuk.model.{CellState, Signal}
import pl.edu.agh.xinuk.model.grid.GridSignalPropagation

import java.awt.Color

object ContinuousEnvMain extends LazyLogging {
  private val configPrefix = "continuous-env"

  def main(args: Array[String]): Unit = {
    import pl.edu.agh.xinuk.config.ValueReaders._
    new Simulation(
      configPrefix,
      GeoKinMetrics.MetricHeaders,
      ContinuousEnvWorldCreator,
      GeoKinPlanCreator,
      GeoKinPlanResolver,
      GeoKinMetrics.empty,
      GridSignalPropagation.Bending,
      cellToColor
    ).start()
  }

  private def cellToColor: PartialFunction[CellState, Color] = {
    case cellState =>
      cellState.contents match {
        // case _ => Color.WHITE
        case continuousEnvCell: ContinuousEnvCell => cellToColorSign(cellState, continuousEnvCell)
      }
  }

  private def cellToColorSign(cellState: CellState, continuousEnvCell: ContinuousEnvCell): Color = {
    if (continuousEnvCell.runners.nonEmpty) {
      Color.RED
    } else if (continuousEnvCell.initialSignal.value > 0) {
      Color.BLUE
    } else if (continuousEnvCell.visited) {
      new Color(0, 128, 0)
    } else {
      val maxSignal = cellState.signalMap
        .values
        .toSeq
        .sortBy(s => -Math.abs(s.value))
        .collectFirst({ case s: Signal => s.value })
        .getOrElse(0d)

      if (maxSignal > 0.75) {
        new Color(64, 64, 255)
      } else if (maxSignal > 0.3) {
        new Color(128, 128, 255)
      } else if (maxSignal > 0.1) {
        new Color(192, 192, 255)
      } else if (maxSignal > 0) {
        new Color(224, 224, 255)
      } else {
        Color.WHITE
      }
    }
  }
}
