package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.geometry.Vec2
import pl.edu.agh.continuous.env.model.{MoveCompletion, Runner}


object RunnerPhysics {
  implicit class RunnerExtensions(val runner: Runner) extends AnyVal {
    def withAppliedForceConsideringLastStep(forceDirection: Vec2,
                                            nextStep: Vec2,
                                            unitAcceleration: Double,
                                            minStepLength: Double,
                                            cellSize: Double): Runner = {
      val maxStepLength = runner.lastMoveCompletion match {
        case Some(value) => if (value >= MoveCompletion.max().safeRounded) {
          runner.maxStepLength(cellSize)
        } else {
          runner.lastActualStep.get.length
        }
        case None => runner.maxStepLength(cellSize)
      }

      runner.withAppliedForceConstrained(forceDirection, nextStep, unitAcceleration, minStepLength, maxStepLength)
    }

    def withAppliedForceConstrained(force: Vec2,
                                    nextStep: Vec2,
                                    unitAcceleration: Double,
                                    minStepLength: Double,
                                    maxStepLength: Double): Runner = {
      val newStep = newStepUnconstrained(force, unitAcceleration)
      val clippedNextStep = nextStep.clipLength(
        lowerBound = minStepLength,
        upperBound = maxStepLength
      )
      runner.withNextStep(clippedNextStep)
    }

    def withAppliedForceConstrainedNoMin(force: Vec2,
                                         unitAcceleration: Double,
                                         cellSize: Double): Runner = {
      val newStep = newStepUnconstrained(force, unitAcceleration)
      val maxStepLength = runner.maxStepLength(cellSize)
      val clippedNextStep = newStep.clipLength(
        lowerBound = 0.0,
        upperBound = maxStepLength
      )
      runner.withNextStep(clippedNextStep)
    }

    private def newStepUnconstrained(force: Vec2,
                                     unitAcceleration: Double): Vec2 =
      runner.nextStep + force * unitAcceleration / runner.mass

    //    def withAppliedForceBackwards(unitAcceleration: Double,
    //                                  minStepLength: Double,
    //                                  cellSize: Double): Runner = {
    //      val maxStepLength = runner.maxStepLength(cellSize)
    //      val newStepLength = runner.nextStep.length - unitAcceleration / runner.mass
    //      val stepLength = newStepLength.clip(lowerBound = minStepLength, upperBound = maxStepLength)
    //      runner.withNextStep(runner.nextStep.normalized * stepLength)
    //    }
  }
}
