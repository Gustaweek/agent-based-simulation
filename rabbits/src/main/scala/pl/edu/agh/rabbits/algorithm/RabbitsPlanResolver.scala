package pl.edu.agh.rabbits.algorithm

import pl.edu.agh.rabbits.algorithm.RabbitsUpdateTag._
import pl.edu.agh.rabbits.config.RabbitsConfig
import pl.edu.agh.rabbits.model.{Environment, Lettuce, Rabbit, Wolf}
import pl.edu.agh.xinuk.algorithm.{PlanResolver, StateUpdate}
import pl.edu.agh.xinuk.model.{CellContents, Empty}

final case class RabbitsPlanResolver() extends PlanResolver[RabbitsConfig] {
  override def isUpdateValid(contents: CellContents, update: StateUpdate)(implicit config: RabbitsConfig): Boolean =
    (contents, update.updateTag, update.value) match {
      case (env: Environment, Stay, _: Lettuce) => env.lettuce.isDefined // Lettuce can Stay in old cell

      case (env: Environment, Stay, _: Rabbit) => env.rabbit.isDefined // Rabbit can Stay in old cell

      case (env: Environment, Stay, _: Wolf) => env.wolf.isDefined // Wolf can Stay in old cell

      case (env: Environment, DieRabbit, Empty) => env.rabbit.isDefined // Rabbit can Die

      case (env: Environment, DieWolf, Empty) => env.wolf.isDefined // Wolf can Die

      case (env: Environment, LeaveRabbit, Empty) => env.rabbit.isDefined // Rabbit can Leave cell

      case (env: Environment, LeaveWolf, Empty) => env.wolf.isDefined // Wolf can Leave cell

      case (env: Environment, Arrive, _: Rabbit) => env.rabbit.isEmpty // Rabbit can Arrive into environment without another rabbit

      case (env: Environment, Arrive, _: Wolf) => env.wolf.isEmpty // Wolf can Arrive into environment without another wolf

      case (env: Environment, Spawn, _: Rabbit) => env.rabbit.isEmpty // Rabbit can Spawn into environment without another rabbit

      case (env: Environment, Spawn, _: Wolf) => env.wolf.isEmpty // Wolf can Spawn into environment without another wolf

      case (env: Environment, Spawn, _: Lettuce) => env.lettuce.isEmpty // Lettuce can Spawn into environment without another lettuce

      case _ => false                               // nothing else is allowed
    }

  override def applyUpdate(contents: CellContents, update: StateUpdate)(implicit config: RabbitsConfig): (CellContents, RabbitsMetrics) = {

    val (newContents: CellContents, metrics: RabbitsMetrics) = (contents, update.updateTag, update.value) match {
      case (env: Environment, Stay, lettuce: Lettuce) =>
        (Environment(env.rabbit, Some(lettuce), env.wolf), RabbitsMetrics.empty)
      case (env: Environment, Stay, rabbit: Rabbit) =>
        (Environment(Some(rabbit), env.lettuce, env.wolf), RabbitsMetrics.empty)
      case (env: Environment, Stay, wolf: Wolf) =>
        (Environment(env.rabbit, env.lettuce, Some(wolf)), RabbitsMetrics.empty)

      case (env: Environment, DieRabbit, Empty) =>
        (Environment(None, env.lettuce, env.wolf), RabbitsMetrics.rabbitDeath(env.rabbit.get))
      case (env: Environment, DieWolf, Empty) =>
        (Environment(env.rabbit, env.lettuce, None), RabbitsMetrics.wolfDeath(env.wolf.get))

      case (env: Environment, LeaveRabbit, Empty) =>
        (Environment(None, env.lettuce, env.wolf), RabbitsMetrics.empty)
      case (env: Environment, LeaveWolf, Empty) =>
        (Environment(env.rabbit, env.lettuce, None), RabbitsMetrics.empty)

      case (env: Environment, Arrive, rabbit: Rabbit) =>
        (Environment(Some(rabbit), env.lettuce, env.wolf), RabbitsMetrics.empty)
      case (env: Environment, Arrive, wolf: Wolf) =>
        (Environment(env.rabbit, env.lettuce, Some(wolf)), RabbitsMetrics.empty)
        //val newRabbit = Rabbit(energy + config.lettuceEnergeticCapacity, lifespan)
        //(newRabbit, RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce))
      //case (Empty, Arrive, rabbit: Rabbit) =>
      //  (rabbit, RabbitsMetrics.rabbit(rabbit))

      case (env: Environment, Spawn, rabbit: Rabbit) =>
        (Environment(Some(rabbit), env.lettuce, env.wolf), RabbitsMetrics.rabbitReproduction)
      case (env: Environment, Spawn, wolf: Wolf) =>
        (Environment(env.rabbit, env.lettuce, Some(wolf)), RabbitsMetrics.wolfReproduction)
        //val newRabbit = Rabbit(energy + config.lettuceEnergeticCapacity, lifespan)
        //(newRabbit, RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce) + RabbitsMetrics.rabbitReproduction)
      //case (Empty, Spawn, rabbit: Rabbit) =>
      //  (rabbit, RabbitsMetrics.rabbit(rabbit) + RabbitsMetrics.rabbitReproduction)

      case (env: Environment, Spawn, lettuce: Lettuce) =>
        (Environment(env.rabbit, Some(lettuce), env.wolf), RabbitsMetrics.empty)
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
      case (Environment(rabbit, lettuce, wolf)) => {
        if (rabbit.isDefined && lettuce.isDefined && wolf.isEmpty) {
          val newRabbit = Rabbit(rabbit.get.energy + config.lettuceEnergeticCapacity, rabbit.get.lifespan)
          (Environment(Some(newRabbit), None, None), RabbitsMetrics.rabbit(newRabbit) + RabbitsMetrics.lettuceConsumed(lettuce.get))
        }
        else if (rabbit.isDefined && lettuce.isDefined && wolf.isDefined) {
          val newWolf = Wolf(wolf.get.energy + rabbit.get.energy + config.lettuceEnergeticCapacity, wolf.get.lifespan)
          (Environment(None, None, Some(newWolf)), RabbitsMetrics.wolf(newWolf) + RabbitsMetrics.lettuceConsumed(lettuce.get)
            + RabbitsMetrics.rabbitConsumed(rabbit.get))
        }
        else if (rabbit.isDefined && lettuce.isEmpty && wolf.isDefined) {
          val newWolf = Wolf(wolf.get.energy + rabbit.get.energy, wolf.get.lifespan)
          (Environment(None, None, Some(newWolf)), RabbitsMetrics.wolf(newWolf) + RabbitsMetrics.rabbitConsumed(rabbit.get))
        }
        else {
          val newEnv = Environment(rabbit, lettuce, wolf)
          var metrics = RabbitsMetrics.empty
          if (rabbit.isDefined) metrics = metrics + RabbitsMetrics.rabbit(rabbit.get)
          if (lettuce.isDefined) metrics = metrics + RabbitsMetrics.lettuce
          if (wolf.isDefined) metrics = metrics + RabbitsMetrics.wolf(wolf.get)
          (newEnv, metrics)
        }
      }
    }
    (newContents, metrics)
  }
}
