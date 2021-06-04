package pl.edu.agh.rabbits.algorithm

import pl.edu.agh.rabbits.algorithm.RabbitsUpdateTag._
import pl.edu.agh.rabbits.config.RabbitsConfig
import pl.edu.agh.rabbits.model.{Environment, Lettuce, Rabbit}
import pl.edu.agh.xinuk.algorithm.{Plan, PlanCreator, Plans}
import pl.edu.agh.xinuk.model._

import scala.math.Ordering
import scala.util.Random

final case class RabbitsPlanCreator() extends PlanCreator[RabbitsConfig] {

  override def createPlans(iteration: Long, cellId: CellId, cellState: CellState, neighbourContents: Map[Direction, CellContents])
                          (implicit config: RabbitsConfig): (Plans, RabbitsMetrics) = {
    cellState.contents match {
      case env: Environment => {
        if (env.lettuce.isDefined) lettucePlanning(iteration, env, neighbourContents)
        else if (env.rabbit.isDefined) rabbitPlanning(env, cellState.signalMap, neighbourContents)
        else (Plans.empty, RabbitsMetrics.empty)
      }
      case _ => (Plans.empty, RabbitsMetrics.empty)
    }
  }

  private def lettucePlanning(iteration: Long, env: Environment, neighbourContents: Map[Direction, CellContents])
                             (implicit config: RabbitsConfig): (Plans, RabbitsMetrics) = {
    val ageLettucePlan = Plan(Stay(Lettuce(env.lettuce.get.lifespan + 1)))

    val spreadMap: Map[Direction, Seq[Plan]] = if (iteration % config.lettuceReproductionFrequency == 0) {
      val availableDirections = neighbourContents.filter {
        case (_, Empty) => true
        case (_, env: Environment) => env.lettuce.isEmpty
        case _ => false
      }.keys.toSeq
      if (availableDirections.nonEmpty) {
        val direction: Direction = availableDirections(Random.nextInt(availableDirections.size))
        val spreadLettucePlan = Plan(Spawn(Lettuce(0)))
        Map((direction, Seq(spreadLettucePlan)))
      } else {
        Map.empty
      }
    } else {
      Map.empty
    }

    (Plans(spreadMap, Seq(ageLettucePlan)), RabbitsMetrics.empty)
  }

  private def rabbitPlanning(env: Environment, signalMap: SignalMap, neighbourContents: Map[Direction, CellContents])
                            (implicit config: RabbitsConfig): (Plans, RabbitsMetrics) = {
    val plans = if (env.rabbit.get.energy < config.rabbitLifeActivityCost) {
      Plans(Map.empty, Seq(Plan(Die())))
    } else {
      val availableDirections = neighbourContents.filter {
        case (_, Empty) => true
        case (_, env: Environment) => env.rabbit.isEmpty
        case _ => false
      }.keys.toSeq

      val agedRabbit = Rabbit(env.rabbit.get.energy - config.rabbitLifeActivityCost, env.rabbit.get.lifespan + 1)

      if (availableDirections.nonEmpty) {
        if (env.rabbit.get.energy > config.rabbitReproductionThreshold) {
          val direction: Direction = availableDirections(Random.nextInt(availableDirections.size))
          Plans(Map((direction, Seq(Plan(
            Spawn(Rabbit(config.rabbitStartEnergy, 0)),
            Stay(Rabbit(env.rabbit.get.energy - config.rabbitReproductionCost, env.rabbit.get.lifespan + 1)),
            Stay(agedRabbit)
          )))))
        } else {
          val direction: Direction = availableDirections.map { direction => (direction, signalMap(direction)) }
            .sortBy(_._2)(Ordering[Signal].reverse).head._1
          Plans(Map((direction, Seq(Plan(
            Arrive(agedRabbit),
            Leave(),
            Stay(agedRabbit)
          )))))
        }
      } else {
        Plans(Map.empty, Seq(Plan(
          Stay(agedRabbit)
        )))
      }
    }

    (plans, RabbitsMetrics.empty)
  }
}