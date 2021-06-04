package pl.edu.agh.rabbits.algorithm

import pl.edu.agh.rabbits.algorithm.RabbitsUpdateTag._
import pl.edu.agh.rabbits.config.RabbitsConfig
import pl.edu.agh.rabbits.model.{Environment, Lettuce, Rabbit}
import pl.edu.agh.xinuk.algorithm.{PlanResolver, StateUpdate}
import pl.edu.agh.xinuk.model.{CellContents, Empty}

final case class RabbitsPlanResolver() extends PlanResolver[RabbitsConfig] {
  override def isUpdateValid(contents: CellContents, update: StateUpdate)(implicit config: RabbitsConfig): Boolean =
    (contents, update.updateTag, update.value) match {
      case (env: Environment, Stay, _: Lettuce) => env.lettuce.isDefined // Lettuce can Stay in old cell

      case (env: Environment, Stay, _: Rabbit) => env.rabbit.isDefined // Rabbit can Stay in old cell

      case (env: Environment, Die, Empty) => env.rabbit.isDefined // Rabbit can Die

      case (env: Environment, Leave, Empty) => env.rabbit.isDefined // Rabbit can Leave cell

      case (env: Environment, Arrive, _: Rabbit) => env.rabbit.isEmpty // Rabbit can Arrive into environment without another rabbit

      case (env: Environment, Spawn, _: Rabbit) => env.rabbit.isEmpty // Rabbit can Spawn into environment without another rabbit

      case (env: Environment, Spawn, _: Lettuce) => env.lettuce.isEmpty // Lettuce can Spawn into environment without another lettuce

      case _ => false                               // nothing else is allowed
    }

  override def applyUpdate(contents: CellContents, update: StateUpdate)(implicit config: RabbitsConfig): (CellContents, RabbitsMetrics) = {

    val (newContents: CellContents, metrics: RabbitsMetrics) = (contents, update.updateTag, update.value) match {
      case (env: Environment, Stay, lettuce: Lettuce) =>
        (Environment(env.rabbit, Some(lettuce)), RabbitsMetrics.lettuce)
      case (env: Environment, Stay, rabbit: Rabbit) =>
        (Environment(Some(rabbit), env.lettuce), RabbitsMetrics.rabbit(rabbit))

      case (env: Environment, Die, Empty) =>
        (Environment(None, env.lettuce), RabbitsMetrics.rabbitDeath(env.rabbit.get))

      case (env: Environment, Leave, Empty) =>
        (Environment(None, env.lettuce), RabbitsMetrics.empty)
      case (env: Environment, Arrive, rabbit: Rabbit) =>
        val newEnv: Environment = Environment(Some(rabbit), env.lettuce)
        if (env.lettuce.isEmpty) (newEnv, RabbitsMetrics.rabbit(rabbit))
        else (newEnv, RabbitsMetrics.empty)
        //val newRabbit = Rabbit(energy + config.lettuceEnergeticCapacity, lifespan)
        //(newRabbit, RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce))
      //case (Empty, Arrive, rabbit: Rabbit) =>
      //  (rabbit, RabbitsMetrics.rabbit(rabbit))

      case (env: Environment, Spawn, rabbit: Rabbit) =>
        val newEnv: Environment = Environment(Some(rabbit), env.lettuce)
        if (env.lettuce.isEmpty) (newEnv, RabbitsMetrics.rabbit(rabbit) + RabbitsMetrics.rabbitReproduction)
        else (newEnv, RabbitsMetrics.rabbitReproduction)
        //val newRabbit = Rabbit(energy + config.lettuceEnergeticCapacity, lifespan)
        //(newRabbit, RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce) + RabbitsMetrics.rabbitReproduction)
      //case (Empty, Spawn, rabbit: Rabbit) =>
      //  (rabbit, RabbitsMetrics.rabbit(rabbit) + RabbitsMetrics.rabbitReproduction)

      case (env: Environment, Spawn, lettuce: Lettuce) =>
        val newEnv: Environment = Environment(env.rabbit, Some(lettuce))
        if (env.rabbit.isEmpty) (newEnv, RabbitsMetrics.lettuce)
        else (newEnv, RabbitsMetrics.empty)
        //val newRabbit = Rabbit(energy + config.lettuceEnergeticCapacity, lifespan)
        //(newRabbit, RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce))
      //case (Empty, Spawn, lettuce: Lettuce) =>
      //  (lettuce, RabbitsMetrics.lettuce)

      case _ => throw new IllegalArgumentException(s"Illegal update applied: contents = $contents, update = $update")
    }

    (newContents, metrics)
  }

  override def resolveKilling(contents: CellContents)(implicit config: RabbitsConfig): (CellContents, RabbitsMetrics) = {
    val (newContents: CellContents, metrics: RabbitsMetrics) = (contents) match {
      case (Environment(rabbit, lettuce)) => {
        if (rabbit.isDefined && lettuce.isDefined) {
          val newRabbit = Rabbit(rabbit.get.energy + config.lettuceEnergeticCapacity, rabbit.get.lifespan)
          (Environment(Some(newRabbit), None), RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce.get))
        }
        else (Environment(rabbit, lettuce), RabbitsMetrics.empty)
      }
    }
    (newContents, metrics)
  }
}
