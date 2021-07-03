package pl.edu.agh.rabbits.algorithm

import pl.edu.agh.rabbits.model.{Lettuce, Rabbit, Wolf}
import pl.edu.agh.xinuk.algorithm.Metrics

final case class RabbitsMetrics(rabbitCount: Long,
                                lettuceCount: Long,
                                wolfCount: Long,
                                rabbitDeaths: Long,
                                wolfDeaths: Long,
                                rabbitTotalEnergy: Double,
                                rabbitReproductionsCount: Long,
                                wolfTotalEnergy: Double,
                                wolfReproductionsCount: Long,
                                consumedLettuceCount: Long,
                                consumedRabbitCount: Long,
                                rabbitTotalLifespan: Long,
                                wolfTotalLifespan: Long,
                                lettuceTotalLifespan: Long) extends Metrics {

  override def log: String = {
    s"$rabbitCount;$wolfCount;$lettuceCount;$rabbitDeaths;$wolfDeaths;$rabbitTotalEnergy;$rabbitReproductionsCount;$wolfTotalEnergy;$wolfReproductionsCount;" +
      s"$consumedLettuceCount;$consumedRabbitCount;$rabbitTotalLifespan;$wolfTotalLifespan;$lettuceTotalLifespan"
  }

  override def series: Vector[(String, Double)] = Vector(
    "Rabbits" -> rabbitCount.toDouble,
    "Lettuce" -> lettuceCount.toDouble,
    "Wolf" -> wolfCount.toDouble
  )

  override def +(other: Metrics): RabbitsMetrics = {
    other match {
      case RabbitsMetrics.Empty => this
      case RabbitsMetrics(otherRabbitCount, otherLettuceCount, otherWolfCount, otherRabbitDeaths, otherWolfDeaths, otherRabbitTotalEnergy,
      otherRabbitReproductionsCount, otherWolfTotalEnergy, otherWolfReproductionsCount, otherConsumedLettuceCount,
      otherConsumedRabbitCount, otherRabbitTotalLifespan, otherWolfTotalLifespan, otherLettuceTotalLifespan) =>
        RabbitsMetrics(
          rabbitCount + otherRabbitCount,
          lettuceCount + otherLettuceCount,
          wolfCount + otherWolfCount,
          rabbitDeaths + otherRabbitDeaths,
          wolfDeaths + otherWolfDeaths,
          rabbitTotalEnergy + otherRabbitTotalEnergy,
          rabbitReproductionsCount + otherRabbitReproductionsCount,
          wolfTotalEnergy + otherWolfTotalEnergy,
          wolfReproductionsCount + otherWolfReproductionsCount,
          consumedLettuceCount + otherConsumedLettuceCount,
          consumedRabbitCount + otherConsumedRabbitCount,
          rabbitTotalLifespan + otherRabbitTotalLifespan,
          wolfTotalLifespan + otherWolfTotalLifespan,
          lettuceTotalLifespan + otherLettuceTotalLifespan)
      case _ => throw new UnsupportedOperationException(s"Cannot add non-RabbitsMetrics to RabbitsMetrics")
    }
  }
}

object RabbitsMetrics {
  val MetricHeaders = Vector(
    "rabbitCount",
    "lettuceCount",
    "wolfCount",
    "rabbitDeaths",
    "wolfDeaths",
    "rabbitTotalEnergy",
    "rabbitReproductionsCount",
    "wolfTotalEnergy",
    "wolfReproductionsCount",
    "consumedLettuceCount",
    "consumedRabbitCount",
    "rabbitTotalLifespan",
    "wolfTotalLifespan",
    "lettuceTotalLifespan"
  )

  private val Empty = RabbitsMetrics(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0)
  private val Lettuce = RabbitsMetrics(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
  private val WolfReproduction = RabbitsMetrics(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0)
  private val RabbitReproduction = RabbitsMetrics(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0)

  def empty: RabbitsMetrics = Empty
  def rabbit(rabbit: Rabbit): RabbitsMetrics = RabbitsMetrics(1, 0, 0, 0, 0, rabbit.energy, 0, 0, 0, 0, 0, 0, 0, 0)
  def wolf(wolf: Wolf): RabbitsMetrics = RabbitsMetrics(0, 0, 1, 0, 0, 0, 0, wolf.energy, 0, 0, 0, 0, 0, 0)
  def lettuce: RabbitsMetrics = Lettuce
  def rabbitDeath(rabbit: Rabbit): RabbitsMetrics = RabbitsMetrics(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, rabbit.lifespan, 0, 0)
  def wolfDeath(wolf: Wolf): RabbitsMetrics = RabbitsMetrics(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, wolf.lifespan, 0)
  def rabbitReproduction: RabbitsMetrics = RabbitReproduction
  def wolfReproduction: RabbitsMetrics = WolfReproduction
  def lettuceConsumed(lettuce: Lettuce): RabbitsMetrics = RabbitsMetrics(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, lettuce.lifespan)
  def rabbitConsumed(rabbit: Rabbit): RabbitsMetrics = RabbitsMetrics(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, rabbit.lifespan, 0, 0)
}