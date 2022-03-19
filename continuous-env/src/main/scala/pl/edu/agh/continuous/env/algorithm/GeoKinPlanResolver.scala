package pl.edu.agh.continuous.env.algorithm

import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, Runner, RunnerOccupied}
import pl.edu.agh.xinuk.algorithm.{Metrics, PlanResolver, StateUpdate}
import pl.edu.agh.xinuk.model.CellContents

import java.util.UUID

final case class GeoKinPlanResolver() extends PlanResolver[ContinuousEnvConfig] {
  override def isUpdateValid(contents: CellContents, update: StateUpdate)(implicit config: ContinuousEnvConfig): Boolean =
    (contents, update.value) match {
      case (ContinuousEnvCell(config.initialSignal), RunnerOccupied(_, _)) => true // Runner finishes
      case (ContinuousEnvCell(_), RunnerOccupied(_, _)) => true // Runners crowd up
      case _ =>
        //reportDiagnostics(InvalidUpdateDiagnostic(left.getClass.getTypeName, right.getClass.getTypeName))
        false // nothing else is allowed
    }

  override def applyUpdate(contents: CellContents, update: StateUpdate)
                          (implicit config: ContinuousEnvConfig): (CellContents, Metrics) = {
    (contents, update.value) match {
      case (continuousEnvCell: ContinuousEnvCell, RunnerOccupied(newGeneration, runners2))
        if runners2.isEmpty && continuousEnvCell.generation + 1 == newGeneration =>
        /*reportDiagnostics(ApplyUpdateDiagnostic(
          leftGeneration = oldGeneration,
          rightGeneration = newGeneration,
          leftRunners = runners1,
          rightRunners = runners2,
          scenario = "leave"))*/
        continuousEnvCell.runners = Array.empty
        continuousEnvCell.coordinates = Map.empty
        continuousEnvCell.generation = 0
        (continuousEnvCell, GeoKinMetrics.empty)
      case (continuousEnvCell: ContinuousEnvCell, RunnerOccupied(newGeneration, runners2))
        if continuousEnvCell.generation == newGeneration =>
        continuousEnvCell.runners = continuousEnvCell.runners ++ runners2.toArray
        continuousEnvCell.coordinates = getRunnersCoords(continuousEnvCell.runners)
        /*reportDiagnostics(ApplyUpdateDiagnostic(
          leftGeneration = oldGeneration,
          rightGeneration = newGeneration,
          leftRunners = runners1,
          rightRunners = runners2,
          scenario = "append"))*/
        (continuousEnvCell, GeoKinMetrics.empty)
      case (continuousEnvCell: ContinuousEnvCell, RunnerOccupied(newGeneration, runners2))
        if continuousEnvCell.generation + 1 == newGeneration =>
        continuousEnvCell.generation = newGeneration
        continuousEnvCell.runners = runners2.toArray
        continuousEnvCell.coordinates = getRunnersCoords(continuousEnvCell.runners)
        /*reportDiagnostics(ApplyUpdateDiagnostic(
          leftGeneration = oldGeneration,
          rightGeneration = newGeneration,
          leftRunners = runners1,
          rightRunners = runners2,
          scenario = "new"))*/
        (continuousEnvCell, GeoKinMetrics.empty)
      case (continuousEnvCell: ContinuousEnvCell, RunnerOccupied(generation, runners)) =>
        if (continuousEnvCell.initialSignal.equals(config.initialSignal)) {
          (continuousEnvCell, GeoKinMetrics.runnersDone(runners.size))
        }
        else if (continuousEnvCell.runners.isEmpty) {
          continuousEnvCell.runners = runners.toArray
          continuousEnvCell.generation = generation
          continuousEnvCell.coordinates = getRunnersCoords(continuousEnvCell.runners)
          (continuousEnvCell, GeoKinMetrics.empty)
        }
        else throw new IllegalArgumentException(s"Illegal update applied: state = $contents, update = $update")
      case _ => throw new IllegalArgumentException(s"Illegal update applied: state = $contents, update = $update")
    }
  }

  private def getRunnersCoords(runners: Array[Runner]): Map[UUID, (Double, Double)] = {
    var result: Map[UUID, (Double, Double)] = Map.empty
    runners.foreach(runner => result += (runner.id -> (runner.position.x, runner.position.y)))
    result
  }
}
