package pl.edu.agh.continuous.env.algorithm

import pl.edu.agh.xinuk.algorithm.Metrics

final case class GeoKinMetrics(runnersCount: Long,
                               runnersDone: Long,
                               runnersFailed: Long) extends Metrics {
  override def log: String = Seq(runnersCount, runnersDone, runnersFailed).mkString(";")

  override def series: Vector[(String, Double)] = Vector(
    "runners count" -> runnersCount.toDouble,
    "runners done" -> runnersDone.toDouble,
    "runners failed" -> runnersFailed.toDouble,
  )

  override def +(other: Metrics): Metrics = {
    other match {
      case GeoKinMetrics.Empty => this
      case GeoKinMetrics(otherRunnersCount, otherRunnersDone, otherRunnersFailed) =>
        GeoKinMetrics(
          runnersCount + otherRunnersCount,
          runnersDone + otherRunnersDone,
          runnersFailed + otherRunnersFailed)
      case _ => throw new UnsupportedOperationException("Cannot add: non-GeoKinMetrics to GeoKinMetrics")
    }
  }
}

object GeoKinMetrics {
  val MetricHeaders = Vector(
    "runnersCount",
    "runnersDone",
    "runnersFailed"
  )

  private val Empty = GeoKinMetrics(0, 0, 0)

  def empty: GeoKinMetrics = Empty
  def runnersDone(count: Long): GeoKinMetrics = GeoKinMetrics(0, count, 0)
  def runnersFailed(count: Long): GeoKinMetrics = GeoKinMetrics(0, 0, count)
}