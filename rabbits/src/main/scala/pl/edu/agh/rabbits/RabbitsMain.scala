package pl.edu.agh.rabbits

import java.awt.Color
import com.typesafe.scalalogging.LazyLogging
import pl.edu.agh.rabbits.algorithm.{RabbitsMetrics, RabbitsPlanCreator, RabbitsPlanResolver, RabbitsWorldCreator}
import pl.edu.agh.rabbits.model.{Environment, Lettuce, Rabbit}
import pl.edu.agh.xinuk.Simulation
import pl.edu.agh.xinuk.model.CellState
import pl.edu.agh.xinuk.model.grid.GridSignalPropagation

object RabbitsMain extends LazyLogging {
  private val configPrefix = "rabbits"

  def main(args: Array[String]): Unit = {
    import pl.edu.agh.xinuk.config.ValueReaders._
    new Simulation(
      configPrefix,
      RabbitsMetrics.MetricHeaders,
      RabbitsWorldCreator,
      () => RabbitsPlanCreator(),
      () => RabbitsPlanResolver(),
      RabbitsMetrics.empty,
      GridSignalPropagation.Standard,
      cellToColor
    ).start()
  }

  private def cellToColor: PartialFunction[CellState, Color] = {
    case cellState =>
      cellState.contents match {
        case env: Environment => {
          if (env.wolf.isDefined) Color.BLACK
          else if (env.rabbit.isDefined) new Color(139, 69, 19)
          else if (env.lettuce.isDefined) new Color(0, 128, 0)
          else Color.WHITE
        }
        case _ => Color.WHITE
      }
  }
}

