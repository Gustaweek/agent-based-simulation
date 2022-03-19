package pl.edu.agh.continuous.env.common

import pl.edu.agh.continuous.env.common.CollisionAvoidance.Exceptions.{CircleMoveCompletionComputationException, GeneralMoveCompletionComputationException, LineMoveCompletionComputationException, MoveCompletionComputationException, ObstaclesMoveCompletionComputationException, OtherRunnersBodiesMoveCompletionComputationException, OtherRunnersMoveCompletionComputationException, OtherRunnersSweptBodiesMoveCompletionComputationException, SideCaseMoveCompletionComputationException, SweptCircleMoveCompletionComputationException}
import pl.edu.agh.continuous.env.common.EquationSolvers.{QuadraticEquationDoubleResult, QuadraticEquationSingleResult, solveQuadraticEquation}
import pl.edu.agh.continuous.env.common.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.geometry.{Circle, Line, SweptCircle, Vec2}
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, MoveCompletion, Runner}
import pl.edu.agh.xinuk.model.{CellContents, Direction}

import java.util.UUID

object CollisionAvoidance {

  implicit class RunnerCollisionAvoidanceExtensions(val runner: Runner) extends AnyVal {
    def getMaxMoveCompletion(allReachableRunners: Seq[Runner],
                             neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                             currentCell: ContinuousEnvCell,
                             cellSize: Double): MoveCompletion = {
      try {
        val minOtherRunnerMoveCompletion = runner.getMaxMoveCompletionConsideringOtherRunners(allReachableRunners)

        val minObstaclesMoveCompletion = runner.getMaxMoveCompletionConsideringObstacles(neighbourContents, currentCell, cellSize)

        Seq(minOtherRunnerMoveCompletion, minObstaclesMoveCompletion).minByValue
      } catch {
        case exception: MoveCompletionComputationException =>
          println(s"An error has occurred while computing move completion for a runner: $runner")
          throw exception
        case exception: Exception =>
          println(s"An error has occurred while computing move completion for a runner: $runner")
          throw GeneralMoveCompletionComputationException(runner, exception)
      }
    }

    def getMaxMoveCompletionConsideringObstacles(neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                                 currentCell: ContinuousEnvCell,
                                                 cellSize: Double): MoveCompletion = {
      try {
        val cells: Map[(ContinuousEnvCell, UUID), Direction] = neighbourContents + ((currentCell, UUID.randomUUID()) -> null)
        ObstacleMapping.NeighborContentsExtensions(cells)
          .mapToObstacleLines(cellSize)
          .map(boundary => (boundary, boundary.intersect(Line(runner.position, runner.position + runner.nextStep))))
          .filter(intersection => intersection._2.nonEmpty)
          .filter(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2)
          .map(intersection => intersection._1)
          .map(boundary => getMaxMoveCompletionRespectingLine(boundary))
          .minByValueOption
          .getOrElse(MoveCompletion.max(tag = "o"))
          .withoutPenalty
      } catch {
        case exception: Exception =>
          throw ObstaclesMoveCompletionComputationException(runner, exception)
      }
    }

    def getMaxMoveCompletionConsideringOtherRunners(allReachableRunners: Seq[Runner]): MoveCompletion = {
      try {
        val bodiesMaxCompletion = getMaxMoveCompletionConsideringOtherRunnersBodies(allReachableRunners)

        val sweptBodiesMaxCompletion = getMaxMoveCompletionConsideringOtherRunnersSweptBodies(allReachableRunners)

        Seq(bodiesMaxCompletion, sweptBodiesMaxCompletion).minByValue
      } catch {
        case exception: Exception =>
          throw OtherRunnersMoveCompletionComputationException(runner, exception)
      }
    }

    def getMaxMoveCompletionConsideringOtherRunnersBodies(runners: Seq[Runner]): MoveCompletion = {
      try {
        val position = runner.position
        val normalToStep = runner.nextStep.normal
        val sidePoint = position + normalToStep

        runners
          .filter(otherRunner => otherRunner.body.intersects(runner.sweptBody))
          .filter(x => isPositionAhead(x.position, runner.nextStep, sidePoint))
          .map(otherRunner => getMaxMoveCompletionRespectingCircle(otherRunner.body))
          .minByValueOption
          .getOrElse(MoveCompletion.max(tag = "d"))
          .withTagPrefix(prefix = "b")
      } catch {
        case exception: Exception =>
          throw OtherRunnersBodiesMoveCompletionComputationException(runner, exception)
      }
    }

    //    private def filterRunnersAhead(otherRunners: Seq[Runner]): Seq[Runner] = {
    //      val position = runner.position
    //      val normalToStep = runner.nextStep.normal
    //      val sidePoint = position + normalToStep
    //      otherRunners.filter(x => isPositionAhead(x.position, runner.nextStep, sidePoint))
    //    }

    def getMaxMoveCompletionConsideringOtherRunnersSweptBodies(runners: Seq[Runner]): MoveCompletion = {
      try {
        val position = runner.position
        val normalToStep = runner.nextStep.normal
        val sidePoint = position + normalToStep

        runners
          .filter(otherRunner => otherRunner.priority > runner.priority) // ignore runners with lower priority
          .filter(otherRunner => otherRunner.sweptBody.intersects(runner.sweptBody)) // only the runners that we are in
          // conflict with
          .filter(x =>
            isPositionAhead(x.position, runner.nextStep, sidePoint) ||
              isPositionAhead(x.endPosition, runner.nextStep, sidePoint))
          .map(otherRunner => runner.getMaxMoveCompletionRespectingInvariantSweptCircle(otherRunner.sweptBody))
          .minByValueOption
          .getOrElse(MoveCompletion.max(tag = "d"))
          .withTagPrefix(prefix = "sc")
      } catch {
        case exception: Exception =>
          throw OtherRunnersSweptBodiesMoveCompletionComputationException(runner, exception)
      }
    }

    private def isPositionAhead(otherRunnerPosition: Vec2, step: Vec2, sidePoint: Vec2): Boolean = {
      val referenceVector = Vec2.apply(from = sidePoint, to = otherRunnerPosition)
      val dot = referenceVector dot step
      dot > 0
    }

    def getMaxMoveCompletionRespectingLine(invariantLine: Line): MoveCompletion = {
      try {
        getMaxMoveCompletionRespectingInvariantSweptCircle(invariantLine.toSweptCircle)
      } catch {
        case exception: Exception =>
          throw LineMoveCompletionComputationException(invariantLine, runner, exception)
      }
    }

    def getMaxMoveCompletionRespectingInvariantSweptCircle(invariantSweptCircle: SweptCircle): MoveCompletion = {
      try {
        if (invariantSweptCircle.intersects(runner.body)) {
          val otherRunner = Runner.createNewMock(invariantSweptCircle)
          val otherRunnerMoveCompletion = otherRunner.getMaxMoveCompletionRespectingCircle(runner.body)
          val otherRunnerMaxMoveCompletionEndPosition = otherRunner.endPosition(otherRunnerMoveCompletion)
          val v = otherRunnerMaxMoveCompletionEndPosition - runner.position
          val w = runner.nextStep
          val cos = (v dot w) / (v.length * w.length)

          if (cos > 0) {
            MoveCompletion.min(tag = "cosmin")
          } else {
            MoveCompletion.max(tag = "cosmax")
          }
        } else {
          val sweptCircleToAdjust = runner.sweptBody
          val d = sweptCircleToAdjust.r + invariantSweptCircle.r

          val invariantSweptCircleLine = invariantSweptCircle.line

          val A = invariantSweptCircleLine.A
          val B = invariantSweptCircleLine.B
          val C = invariantSweptCircleLine.C

          val D = d * Math.sqrt(Seq(A, B).map(x => x * x).sum)
          val E = -A * sweptCircleToAdjust.start.x - B * sweptCircleToAdjust.start.y - C
          val F = A * sweptCircleToAdjust.line.dx + B * sweptCircleToAdjust.line.dy

          if (F ~= 0) {
            getSideCaseCompletion(invariantSweptCircle, sweptCircleToAdjust.start)
          } else {
            val c1 = (D + E) / F
            val c2 = (-D + E) / F

            val moveCompletionA = Seq(c1, c2)
              .zipWithIndex
              .map { case (c, i) => (c.abs, i) }
              .flatMap { case (c, i) => MoveCompletion.tryApply(c, tag = s"c$i") }
              .minByOption(_.value)

            moveCompletionA match {
              case Some(value) =>
                val cPoint = runner.endPosition(value)
                val cProjected = invariantSweptCircleLine.pointProjection(cPoint)

                if (invariantSweptCircleLine.segmentContains(cProjected)) {
                  value.withNormal(Vec2(
                    from = cProjected,
                    to = runner.endPosition(value)
                  ))
                } else {
                  getSideCaseCompletion(invariantSweptCircle, cProjected).withTagPrefix(prefix = "nsc")
                }
              case None =>
                getSideCaseCompletion(invariantSweptCircle, sweptCircleToAdjust.start).withTagPrefix(prefix = "n")
            }
          }
        }
      } catch {
        case exception: Exception =>
          throw SweptCircleMoveCompletionComputationException(invariantSweptCircle, runner, exception)
      }
    }

    private def getSideCaseCompletion(invariantSweptCircle: SweptCircle, point: Vec2): MoveCompletion = {
      try {
        val (closestEnd, side) = Seq(invariantSweptCircle.line.start, invariantSweptCircle.line.end)
          .zipWithIndex
          .minBy { case (vec, _) => Line(vec, point).lengthSq }

        getMaxMoveCompletionRespectingCircle(Circle(closestEnd, invariantSweptCircle.r))
          .withTagPrefix(prefix = s"si$side")
      } catch {
        case exception: Exception =>
          throw SideCaseMoveCompletionComputationException(invariantSweptCircle, point, runner, exception)
      }
    }

    def getMaxMoveCompletionRespectingCircle(invariantCircle: Circle): MoveCompletion = {
      try {
        val AB = runner.sweptBody.line
        val A = AB.start
        val B = AB.end
        val C = invariantCircle.center

        val alphaX = C.x - A.x
        val alphaY = C.y - A.y
        val betaX = B.x - A.x
        val betaY = B.y - A.y
        val r = runner.radius
        val R = invariantCircle.r
        val l = r + R

        val a = betaX * betaX + betaY * betaY
        val b = -2.0 * (alphaX * betaX + alphaY * betaY)
        val c = alphaX * alphaX + alphaY * alphaY - l * l

        solveQuadraticEquation(a, b, c) match {
          case Left(value) => throw new Exception(s"Quadratic equation solving failed: $value")
          case Right(value) =>
            val moveCompletion = value match {
              case dr@QuadraticEquationDoubleResult(_, _) => MoveCompletion(
                dr.filter(MoveCompletion.isValueWithinBounds).min, tag = "c_q2")
              case QuadraticEquationSingleResult(result) => MoveCompletion(result.abs, tag = "c_q1")
            }

            moveCompletion.withNormal(Vec2(
              from = invariantCircle.center,
              to = runner.endPosition(moveCompletion)
            ))
        }
      } catch {
        case exception: Exception =>
          throw CircleMoveCompletionComputationException(invariantCircle, runner, exception)
      }
    }
  }

  object Exceptions {
    abstract class MoveCompletionComputationException(message: String,
                                                      runner: Runner,
                                                      cause: Exception)
      extends Exception(s"Move completion computation failed for runner $runner: $message", cause)

    case class GeneralMoveCompletionComputationException(runner: Runner,
                                                         cause: Exception)
      extends MoveCompletionComputationException(
        "Failed to compute move completion for runner", runner, cause)

    case class ObstaclesMoveCompletionComputationException(runner: Runner,
                                                           cause: Exception)
      extends MoveCompletionComputationException(
        "Failed to compute move completion respecting obstacles", runner, cause)

    case class OtherRunnersBodiesMoveCompletionComputationException(runner: Runner,
                                                                    cause: Exception)
      extends MoveCompletionComputationException(
        "Failed to compute move completion respecting other runners bodies", runner, cause)

    case class OtherRunnersSweptBodiesMoveCompletionComputationException(runner: Runner,
                                                                         cause: Exception)
      extends MoveCompletionComputationException(
        "Failed to compute move completion respecting other runners swept bodies", runner, cause)

    case class OtherRunnersMoveCompletionComputationException(runner: Runner,
                                                              cause: Exception)
      extends MoveCompletionComputationException(
        "Failed to compute move completion respecting other runners", runner, cause)

    case class LineMoveCompletionComputationException(line: Line,
                                                      runner: Runner,
                                                      cause: Exception)
      extends MoveCompletionComputationException(
        s"Failed to compute move completion respecting line: $line", runner, cause)

    case class CircleMoveCompletionComputationException(circle: Circle,
                                                        runner: Runner,
                                                        cause: Exception)
      extends MoveCompletionComputationException(
        s"Failed to compute move completion respecting circle: $circle", runner, cause)

    case class SweptCircleMoveCompletionComputationException(sweptCircle: SweptCircle,
                                                             runner: Runner,
                                                             cause: Exception)
      extends MoveCompletionComputationException(
        s"Failed to compute move completion respecting swept circle $sweptCircle", runner, cause)

    case class SideCaseMoveCompletionComputationException(sweptCircle: SweptCircle,
                                                          point: Vec2,
                                                          runner: Runner,
                                                          cause: Exception)
      extends MoveCompletionComputationException(
        s"Failed to compute move completion for side case ($sweptCircle, $point)", runner, cause)
  }
}
