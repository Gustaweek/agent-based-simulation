package pl.edu.agh.continuous.env.algorithm

import pl.edu.agh.continuous.env.common.CellBoundedPosition.PositionExtensions
import pl.edu.agh.continuous.env.common.CollisionAvoidance.RunnerCollisionAvoidanceExtensions
import pl.edu.agh.continuous.env.common.MathUtils.DoubleExtensions
import pl.edu.agh.continuous.env.common.ObstacleMapping
import pl.edu.agh.continuous.env.common.RunnerPhysics.RunnerExtensions
import pl.edu.agh.continuous.env.common.ToVec2Conversions.SignalMapConversionExtensions
import pl.edu.agh.continuous.env.common.geometry.Algorithms.LineIntersection
import pl.edu.agh.continuous.env.common.geometry.{Line, Vec2}
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, MoveCompletion, Runner, RunnerOccupied}
import pl.edu.agh.xinuk.algorithm.{Plan, PlanCreator, Plans, StateUpdate}
import pl.edu.agh.xinuk.model._
import pl.edu.agh.xinuk.model.continuous._
import pl.edu.agh.xinuk.model.grid.GridDirection
import pl.edu.agh.xinuk.model.grid.GridDirection.{BottomLeft, BottomRight, TopLeft, TopRight}

import java.util.{NoSuchElementException, UUID}
import scala.collection.mutable.ListBuffer

final case class GeoKinPlanCreator() extends PlanCreator[ContinuousEnvConfig] {

  override def createPlans(iteration: Long,
                           cellId: CellId,
                           cellState: CellState,
                           neighbourhoodState: NeighbourhoodState)
                          (implicit config: ContinuousEnvConfig): (Plans, GeoKinMetrics) = {
    cellState.contents match {
      case continuousEnvCell: ContinuousEnvCell =>
        if (continuousEnvCell.runners.nonEmpty) {
          var neighboursMap: Map[GridDirection, Iterable[(ContinuousEnvCell, UUID)]] = neighbourhoodState.diagonalNeighbourhoodState
            .filter { case (_, cellState) => !cellState.equals(CellState.empty()) }
            .map(cell => (cell._1, Iterable.single((cell._2.contents.asInstanceOf[ContinuousEnvCell]), UUID.randomUUID())))
          neighboursMap ++= neighbourhoodState.cardinalNeighbourhoodState
            .map(state => (state._1, mapBoundaryStateToCells2(state._2)))
          val reverseNeighboursMap = for ((k, v) <- neighboursMap) yield (v, k)
          val flattenedNeighboursMap: Map[(ContinuousEnvCell, UUID), GridDirection] = reverseNeighboursMap.flatMap { case (k, v) => k.map(_ -> v) }

          val gridCellId = cellId.asInstanceOf[GridMultiCellId]
          val allReachableRunners = getAllReachableRunners(continuousEnvCell, flattenedNeighboursMap, config)
          //reportDiagnostics(AllReachableRunnersDiagnostic(gridCellId, allReachableRunners, ro.runners))
          assertNoOverlaps(iteration, gridCellId, allReachableRunners)
          if (iteration % 2 == 0) {
            (moveRunnersFromCell(gridCellId, continuousEnvCell, flattenedNeighboursMap, neighbourhoodState, allReachableRunners, config),
              GeoKinMetrics.empty)
          } else {
            (adjustVelocityInCell(continuousEnvCell, cellState.signalMap, flattenedNeighboursMap, allReachableRunners.toSet, config),
              GeoKinMetrics.empty)
          }
        }
        else {
          (Plans.empty, GeoKinMetrics.empty)
        }
      case _ => (Plans.empty, GeoKinMetrics.empty)
    }
  }

  private def mapBoundaryStateToCells2(state: BoundaryState): Iterable[(ContinuousEnvCell, UUID)] = {
    state.boundaryStates.values
      .map(cell => (cell.contents.asInstanceOf[ContinuousEnvCell], UUID.randomUUID()))
  }

  private def mapBoundaryStateToCells(state: BoundaryState): Iterable[ContinuousEnvCell] = {
    state.boundaryStates.values
      .map(cell => cell.contents.asInstanceOf[ContinuousEnvCell])
  }

  private def assertNoOverlaps(iteration: Long,
                               cellId: GridMultiCellId,
                               allReachableRunners: Array[Runner]): Unit = {
    for {
      (r1, r1idx) <- allReachableRunners.zipWithIndex
      (r2, r2idx) <- allReachableRunners.zipWithIndex
      if r1idx < r2idx
    } {
      val r1Body = r1.body
      val r2Body = r2.body
      val bodiesIntersect = r1Body.intersects(r2Body)
      if (bodiesIntersect) {
        //reportDiagnostics(RunnerInCollisionDiagnostic(r1))
        //reportDiagnostics(RunnerInCollisionDiagnostic(r2))
        throw new IllegalStateException(s"Iteration[$iteration] Cell(${cellId.x}, ${cellId.y}) " +
          s"Runners $r1 and $r2 collide with each other!")
      }
    }
  }

  private def adjustVelocityInCell(cell: ContinuousEnvCell,
                                   signalMap: SignalMap,
                                   neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                   allReachableRunners: Set[Runner],
                                   config: ContinuousEnvConfig): Plans = {
    val runnersWithAdjustedVelocity = cell.runners.toSet
      .map(runner => adjustVelocityForRunner(
        runner,
        signalMap,
        cell,
        neighbourContents,
        config))
      .map(runner => adjustNextStepToObstacles(
        runner,
        neighbourContents,
        cell,
        allReachableRunners - runner,
        config,
        stage = 1))
      .map(runner => adjustNextStepWithRepellingForce(
        runner,
        cell,
        neighbourContents,
        config))
      .map(runner => adjustNextStepToObstacles(
        runner,
        neighbourContents,
        cell,
        allReachableRunners - runner,
        config,
        stage = 2))
    new Plans(Map.empty, Seq(Plan(StateUpdate(RunnerOccupied(cell.generation + 1, runnersWithAdjustedVelocity)))))
  }

  private def adjustVelocityForRunner(runner: Runner,
                                      signalMap: SignalMap,
                                      cell: ContinuousEnvCell,
                                      neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                      config: ContinuousEnvConfig): Runner = {
    if (runner.path.isEmpty) {
      val force = signalMap.toVec2.normalized
      if (force.length != 0.0) {
        val destinationLine = Line(runner.position, Vec2(runner.position.x + force.x * config.cellSize * 2.0,
          runner.position.y + force.y * config.cellSize * 2.0))
        val destination = adjustDestination(destinationLine, cell, neighbourContents, config.cellSize)
        if (cell.graph.isEmpty) {
          runner.path = List(runner.position, destination)
        }
        else {
          runner.path = findPath(runner.position, destination, cell.graph, config.cellSize).toList
        }
      }
    }

    var nextStep = Vec2.zero
    if (runner.path.nonEmpty) {
      var target = Vec2.zero
      try {
        target = findNextStep(runner.path, runner.position, cell, neighbourContents, config.cellSize)
      }
      catch {
        case _: NoSuchElementException => runner.path = findPath(runner.position, runner.path.last, cell.graph, config.cellSize).toList
          target = findNextStep(runner.path, runner.position, cell, neighbourContents, config.cellSize)
      }
      val movementVector = Line(runner.position, target)
      if (movementVector.length <= runner.speed) {
        nextStep = Vec2(movementVector.end.x - movementVector.start.x, movementVector.end.y - movementVector.start.y)
      }
      else {
        nextStep = Vec2((movementVector.end.x - movementVector.start.x) / movementVector.length * runner.speed,
          (movementVector.end.y - movementVector.start.y) / movementVector.length * runner.speed)
      }
    }

    val adjustedRunner = runner //TODO acceleration must be increased by cellsize^2
      .withNewPriority()
      .withIncrementedGeneration()
      .withAppliedForceConsideringLastStep(
        nextStep,
        config.personUnitAcceleration,
        config.personMinStepLength,
        config.cellSize)
    //val adjustedRunnerForDiagonalMovement = adjustNextStepForDiagonalNeighbourhood(adjustedRunner, neighbourContents, cell, config.cellSize)

    reportPossibleFulfillmentDiagnostics(adjustedRunner, config.cellSize)
    adjustedRunner
  }

  private def adjustDestination(destinationLine: Line,
                                cell: ContinuousEnvCell,
                                neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                cellSize: Int): Vec2 = {
    //val destination = limitLineToNeighbourObstacles(destinationLine, neighbourContents, cellSize)
    val destination = destinationLine
    var targetSegmentVertice = Vec2.zero
    if (destination.end.x <= cellSize.doubleValue && destination.end.x >= 0.0 &&
      destination.end.y <= cellSize.doubleValue && destination.end.y >= 0.0) {
      val targetSegment = cell.cardinalSegments.minBy(segmentEntry =>
        Math.min(Line(destination.end, segmentEntry._1.start).length,
          Line(destination.end, segmentEntry._1.end).length))._1

      if (Line(destination.end, targetSegment.start).length <
        Line(destination.end, targetSegment.end).length) {
        targetSegmentVertice = targetSegment.start
      }
      else {
        targetSegmentVertice = targetSegment.end
      }
      val destinationDir = getDirectionForCoords(targetSegment.center, cellSize.doubleValue)
      val neighbour = getNeighbourForLine(neighbourContents, targetSegment, cellSize)
      if (neighbour.graph.nonEmpty) {
        return neighbour.graph
          .map(graphVertice => graphVertice._1.cellBounded(cellSize, true).adjust(destinationDir, true))
          .minBy(graphVertice => Line(graphVertice, targetSegmentVertice).length)
      }
      return destination.start + Vec2(destination.start, targetSegmentVertice) * 1.5
    }

    val destinationDir = getDirectionForCoords(destination.end, cellSize.doubleValue)
    var neighbour: ContinuousEnvCell = cell
    var targetSegment = (Line(Vec2.zero, Vec2.zero), Vec2.zero)
    var sendToDiagonal: Boolean = false
    if (destinationDir.isDiagonal) {
      neighbour = neighbourContents.filter(cell => cell._2.equals(destinationDir))
        .map(cell => cell._1._1)
        .head
      //we search for segment crossed to reach diagonal neighbour
      //if potentialTargetSegments is empty, it means the original destination vector leads outside of diagonal neighbour's cellOutline
      //so we must give up on it and search for a segment which was crossed first to reach one of cardinal neighbours
      //and will send agent to respective cardinal neighbour rather than non-existent diagonal one
      val adjustedNeighbourSegments = neighbour.cardinalSegments.keys
        .map(segment => Line(segment.start.cellBounded(cellSize, true).adjust(destinationDir, true),
          segment.end.cellBounded(cellSize, true).adjust(destinationDir, true)))
      val potentialTargetSegments = crossedNeighbourSegments(destination, adjustedNeighbourSegments)
      if (potentialTargetSegments.nonEmpty) {
        targetSegment = potentialTargetSegments
          .minBy(crossedSegment => Line(crossedSegment._2, destination.end).length)
        //in the line above we search crossed segments for which point of crossing is nearest to the position of agent for whom we adjust destination
        sendToDiagonal = true
        if (neighbour.graph.nonEmpty) {
          var point = Vec2.zero
          if (destinationDir.equals(BottomLeft)) point = Vec2(0, 0)
          if (destinationDir.equals(BottomRight)) point = Vec2(cellSize, 0)
          if (destinationDir.equals(TopLeft)) point = Vec2(0, cellSize)
          if (destinationDir.equals(TopRight)) point = Vec2(cellSize, cellSize)
          return neighbour.graph
            .map(graphVertice => graphVertice._1.cellBounded(cellSize, true).adjust(destinationDir, true))
            .minBy(graphVertice => Line(graphVertice, point).length)
        }
      }
    }
    if (!sendToDiagonal) {
      targetSegment = crossedNeighbourSegments(destination, cell.cardinalSegments.keys)
        .minBy(crossedSegment => Line(crossedSegment._2, destination.start).length)
      neighbour = getNeighbourForLine(neighbourContents, targetSegment._1, cellSize)
    }

    if (neighbour.graph.nonEmpty) {
      return neighbour.graph
        .map(graphVertice => graphVertice._1.cellBounded(cellSize, true).adjust(destinationDir, true))
        .minBy(graphVertice => Line(graphVertice, targetSegment._2).length)
    }
    destination.end
  }

  private def getDirectionForCoords(coords: Vec2, cellSize: Double): GridDirection = {
    if (coords.x <= 0.0) {
      if (coords.y <= 0.0) {
        return GridDirection.BottomLeft
      }
      if (coords.y >= cellSize) {
        return GridDirection.TopLeft
      }
      return GridDirection.Left
    }
    if (coords.x >= cellSize) {
      if (coords.y <= 0.0) {
        return GridDirection.BottomRight
      }
      if (coords.y >= cellSize) {
        return GridDirection.TopRight
      }
      return GridDirection.Right
    }
    if (coords.y <= 0.0) {
      return GridDirection.Bottom
    }
    GridDirection.Top
  }

  /*private def getDirectionForCoords(coords: Vec2, cellSize: Double): GridDirection = {
    if (coords.x <= 0.0) {
      if (coords.y <= 0.0) {
        return GridDirection.TopRight
      }
      if (coords.y >= cellSize) {
        return GridDirection.TopLeft
      }
      return GridDirection.Top
    }
    if (coords.x >= cellSize) {
      if (coords.y <= 0.0) {
        return GridDirection.BottomRight
      }
      if (coords.y >= cellSize) {
        return GridDirection.BottomLeft
      }
      return GridDirection.Bottom
    }
    if (coords.y <= 0.0) {
      return GridDirection.Right
    }
    GridDirection.Left
  }*/

  private def getNeighbourForLine(neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                  segment: Line,
                                  cellSize: Int): ContinuousEnvCell = {
    /*.map(cell => cell._1._1.cardinalSegments.start.cellBounded(cellSize, true).adjust(destinationDir, true),
      segment.end.cellBounded(cellSize, true).adjust(destinationDir, true))*/
    val dirNeighbours: Map[(ContinuousEnvCell, UUID), Iterable[Line]] = neighbourContents
      .filter(cell => cell._2.asInstanceOf[GridDirection].isCardinal)
      .map(cell => (cell._1, cell._1._1.cardinalSegments
        .map(segment => Line(segment._1.start.cellBounded(cellSize, true).adjust(cell._2, true),
          segment._1.end.cellBounded(cellSize, true).adjust(cell._2, true)))))

    val value1 = dirNeighbours.toList
      .filter(neighbour => {
        val value = neighbour._2
        val set = value
          .toSet
        val contains = set.
          contains(segment)
        contains
      })
      .map(_._1)
    value1.head._1
    /*
    dirNeighbours
      .filter(neighbour => neighbour._2.toList.contains(segment))
      .keys.head._1

    val dirNeighbours: List[ContinuousEnvCell] = neighbourContents
      .filter(cell => cell._2.asInstanceOf[GridDirection].isCardinal)
      .map(cell => cell._1._1)
      .toList
    dirNeighbours
      .filter(neighbour => neighbour.cardinalSegments.exists(entry => entry._1.equals(segment)))
      .head*/
  }

  private def findNextStep(path: List[Vec2],
                           startingPosition: Vec2,
                           cell: ContinuousEnvCell,
                           neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                           cellSize: Int): Vec2 = {
    val lines = ObstacleMapping.NeighborContentsExtensions(neighbourContents + ((cell, UUID.randomUUID()) -> null))
      .mapToObstacleLines(cellSize).toList
    var result = startingPosition
    var flag = false
    path.reverse.foreach(vertice => {
      val intersections: List[(Line, Option[LineIntersection])] = lines
        .map(line => (line, line.intersect(Line(startingPosition, vertice))))
        .filter(intersection => intersection._2.nonEmpty)
        .filter(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2)
      if (intersections.isEmpty && !flag) {
        result = vertice
        flag = true
      }
    })
    result
    /*path.findLast(vertice => !ObstacleMapping.NeighborContentsExtensions(neighbourContents + ((cell, UUID.randomUUID()) -> null))
      .mapToObstacleLines(cellSize)
      .map(line => (line, line.intersect(Line(startingPosition, vertice))))
      .filter(intersection => intersection._2.nonEmpty)
      .exists(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2))
      .get*/
    //replaced head with get
  }

  private def limitLineToNeighbourObstacles(destinationLine: Line,
                                            neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                            cellSize: Int): Line = {
    var nearestCollisionPoint: Vec2 = Vec2.zero
    try {
      nearestCollisionPoint = ObstacleMapping.NeighborContentsExtensions(neighbourContents)
        .mapToObstacleLines(cellSize)
        .map(line => line.intersect(destinationLine))
        .filter(intersection => intersection.nonEmpty)
        .filter(intersection => intersection.get.onLine1 && intersection.get.onLine2)
        .map(intersection => intersection.get.pos)
        .minBy(intersectionPoint => Line(destinationLine.start, intersectionPoint).length)
    }
    catch {
      case _: UnsupportedOperationException => return destinationLine //there were no obstacles blocking destinationLine
    }
    val newLineLength: Double = Line(destinationLine.start, nearestCollisionPoint).length
    val normalized: Vec2 = Vec2((nearestCollisionPoint.x - destinationLine.start.x) / newLineLength,
      (nearestCollisionPoint.y - destinationLine.start.y) / newLineLength)
    Line(destinationLine.start, Vec2(destinationLine.start.x + normalized.x * newLineLength * 0.99,
      destinationLine.start.y + normalized.y * newLineLength * 0.99))
  }

  private def findPath(start: Vec2, end: Vec2, graph: Map[Vec2, Set[Vec2]], cellSize: Int): ListBuffer[Vec2] = {
    val closestStart: Vec2 = graph.minBy(v => Line(v._1, start).length)._1
    val closestEnd: Vec2 = graph.minBy(v => Line(v._1, end).length)._1
    var adjustedGraph: Map[Vec2, Set[Vec2]] = graph
    adjustedGraph = adjustedGraph + (start -> Set(closestStart))
    adjustedGraph = adjustedGraph.updatedWith(closestStart)({ case Some(verticeNeighbours) => Some(verticeNeighbours ++ Set(start))
    case _ => throw new RuntimeException("could not update graph")
    })
    adjustedGraph = adjustedGraph + (end -> Set(closestEnd))
    adjustedGraph = adjustedGraph.updatedWith(closestEnd)({ case Some(verticeNeighbours) => Some(verticeNeighbours ++ Set(end))
    case _ => throw new RuntimeException("could not update graph")
    })

    val discoveredNodes: scala.collection.mutable.Set[Vec2] = scala.collection.mutable.Set(start)
    val cameFrom: scala.collection.mutable.Map[Vec2, Vec2] = scala.collection.mutable.Map.empty
    val discoveredPathCost: scala.collection.mutable.Map[Vec2, Double] = scala.collection.mutable.Map.empty
    discoveredPathCost.put(start, 0.0)
    val totalPathEstimatedCost: scala.collection.mutable.Map[Vec2, Double] = scala.collection.mutable.Map.empty
    totalPathEstimatedCost.put(start, Line(start, end).length) //heuristic function is simply distance

    while (discoveredNodes.nonEmpty) {
      val currentNode: Vec2 = discoveredNodes.minBy(node => totalPathEstimatedCost.get(node).orElse(Some(cellSize.doubleValue * 10.0)))
      if (currentNode.x == end.x && currentNode.y == end.y) {
        return reconstructPath(cameFrom, currentNode)
      }
      discoveredNodes -= currentNode
      adjustedGraph(currentNode).foreach(neighbourNode => {
        val currentDistance: Double = discoveredPathCost(currentNode) + Line(currentNode, neighbourNode).length
        if (currentDistance < discoveredPathCost.get(neighbourNode).orElse(Some(cellSize.doubleValue * 10.0)).get) {
          cameFrom.put(neighbourNode, currentNode)
          discoveredPathCost.put(neighbourNode, currentDistance)
          totalPathEstimatedCost.put(neighbourNode, currentDistance + Line(neighbourNode, end).length)
          discoveredNodes += neighbourNode
        }
      })
    }
    throw new RuntimeException("Failed to find path for graph: " + adjustedGraph.toString())
  }

  private def reconstructPath(cameFrom: scala.collection.mutable.Map[Vec2, Vec2],
                              current: Vec2): ListBuffer[Vec2] = {
    val path: ListBuffer[Vec2] = ListBuffer(current)
    var currentNode: Vec2 = current
    while (cameFrom.contains(currentNode)) {
      currentNode = cameFrom.get(currentNode).orNull
      path.prepend(currentNode)
    }
    path
  }

  private def reportPossibleFulfillmentDiagnostics(runner: Runner,
                                                   cellSize: Double): Unit = {
    val maxStepLength = runner.maxStepLength(cellSize)

    /*reportDiagnostics(RunnerPossibleStepFulfillment(
      runner = runner,
      maxStepLength = maxStepLength,
      normalizedStepLength = runner.nextStep.length
    ))*/
  }

  private def adjustNextStepForDiagonalNeighbourhood(runner: Runner,
                                                     neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                                     cell: ContinuousEnvCell,
                                                     cellSize: Int): Runner = {
    val runnerStep: Line = Line(runner.position, runner.position + runner.nextStep)
    val crossedLines = crossedNeighbourSegments(runnerStep, cell.cardinalSegments.keys)
      .map(crossedLine => crossedLine._1)
    if (crossedLines.isEmpty) {
      return runner //if runner doesn't cross any segment it means it stays in current cell and no movement limiting is needed
    }

    //below we need to adjust coords of neighbour segments to be relative to current cell, otherwise
    //every segment will be within 0-100 in both x and y which will lead to faulty algorithm results
    val crossedLinesOnNeighbourSide = crossedNeighbourSegments(runnerStep, adjustNeighbourSegmentsCoords(neighbourContents, cellSize))
    val diagonalNeighbourSegments = getDiagonalNeighboursSegments(neighbourContents, cellSize)
    val invalidCrossingPoints = crossedLinesOnNeighbourSide.filter(crossedLine => !crossedLines.contains(crossedLine._1) &&
      (diagonalNeighbourSegments.isEmpty || !diagonalNeighbourSegments.contains(crossedLine._1)))
      .map(crossedLine => crossedLine._2)
    //above we filter any boundary segments of neighbours agent crossed either when leaving original cell or entering diagonal neighbour
    // - both cases are valid movements, if there are segments that match neither of the above cases,
    //it means the agent first left its starting cell, went through neighbour and then left this neighbour to another one, however since
    //the final destination wasn't a diagonal neighbour, it must've been a neighbour's neighbour, so the movement is invalid and must be limited
    if (invalidCrossingPoints.nonEmpty) {
      val crossingPoint = invalidCrossingPoints.head
      val newNextStepLength = Line(runner.position, crossingPoint).length
      val newNextStep = Vec2(crossingPoint.x - runner.position.x, crossingPoint.y - runner.position.y)
        .normalized * newNextStepLength * 0.99
      return runner.withNextStep(newNextStep)
    }
    runner
  }

  private def adjustNeighbourSegmentsCoords(neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                            cellSize: Int): scala.collection.mutable.ListBuffer[Line] = {
    val adjustedNeighbourSegments: scala.collection.mutable.ListBuffer[Line] = scala.collection.mutable.ListBuffer.empty
    neighbourContents
      .foreach(neighbourEntry => adjustedNeighbourSegments.addAll(neighbourEntry._1._1.cardinalSegments.keys
        .map(segment =>
          Line(segment.start.cellBounded(cellSize, true).adjust(neighbourEntry._2, true),
            segment.end.cellBounded(cellSize, true).adjust(neighbourEntry._2, true)))))
    adjustedNeighbourSegments
  }

  private def crossedNeighbourSegments(crossingLine: Line,
                                       segments: Iterable[Line]): List[(Line, Vec2)] = {
    segments
      .map(segment => (segment, segment.intersect(crossingLine)))
      .filter(intersection => intersection._2.nonEmpty)
      .filter(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2)
      .map(intersection => (intersection._1, intersection._2.get.pos))
      .toList
  }

  private def getDiagonalNeighboursSegments(neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                            cellSize: Int): List[Line] = {
    adjustNeighbourSegmentsCoords(neighbourContents
      .filter(entry => entry._2.asInstanceOf[GridDirection].isDiagonal), cellSize)
      .toList
  }

  private def adjustNextStepToObstacles(runner: Runner,
                                        neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                        currentCell: ContinuousEnvCell,
                                        allReachableRunners: Set[Runner],
                                        config: ContinuousEnvConfig,
                                        stage: Int): Runner = {
    val inflatedRunners = inflateRunners(allReachableRunners.toSeq)

    val moveCompletionConsideringObstacles =
      tryGetMaxMoveCompletionOrMin(() =>
        runner.getMaxMoveCompletionConsideringObstacles(neighbourContents, currentCell, config.cellSize))
    val moveCompletionConsideringRunnersBodies =
      tryGetMaxMoveCompletionOrMin(() =>
        runner.getMaxMoveCompletionConsideringOtherRunnersBodies(inflatedRunners))
    val maxMoveCompletion = Seq(
      moveCompletionConsideringRunnersBodies,
      moveCompletionConsideringObstacles
    ).minByValue

    if (maxMoveCompletion < MoveCompletion.max()) {
      maxMoveCompletion.normal match {
        case Some(value) =>
          //reportDiagnostics(RunnerStepComponentAdjustmentDiagnostic(runner, maxMoveCompletion, stage))
          val component = runner.nextStep.projectionOn(value) * (MoveCompletion.max().value - maxMoveCompletion.value)
          val twistedNextStep = (runner.nextStep - component).normalized * runner.nextStep.length

          val maxStepLength = runner.maxStepLength(config.cellSize)
          val directionDelta = twistedNextStep.angle - runner.nextStep.angle
          if (directionDelta ~= 0.0) {
            runner
          } else {
            val cosDirectionDelta = math.cos(directionDelta)
            val maxTwistedNextStepLength =
              (maxStepLength * math.sqrt(2) * math.sqrt(1 - cosDirectionDelta)) / (2 - 2 * cosDirectionDelta)

            runner.withNextStep(twistedNextStep.clipLength(lowerBound = 0, upperBound = maxTwistedNextStepLength))
          }
        case None =>
          //reportDiagnostics(RunnerEmptyStepComponentAdjustmentDiagnostic(runner, maxMoveCompletion, stage))
          runner
      }
    } else {
      runner
    }
  }

  private def adjustNextStepWithRepellingForce(runner: Runner,
                                               currentCell: ContinuousEnvCell,
                                               neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                               config: ContinuousEnvConfig): Runner = {
    /*val toBottom = Vec2(1, 0)
    val toTop = -toBottom
    val toLeft = Vec2(0, 1)
    val toRight = -toLeft*/
    val toBottom = Vec2(0, -1)
    val toTop = -toBottom
    val toLeft = Vec2(-1, 0)
    val toRight = -toLeft

    val cells: Map[(ContinuousEnvCell, UUID), Direction] = neighbourContents + ((currentCell, UUID.randomUUID()) -> null)
    val obstacleSegments: Iterable[Line] = ObstacleMapping.NeighborContentsExtensions(cells)
      .mapToObstacleLines(config.cellSize)
      .filter(line => line.segmentDistance(runner.position + runner.nextStep) < 0.1)
    if (obstacleSegments.isEmpty) {
      runner
    }
    else {
      val repellingForceBase = obstacleSegments.flatMap(segment => getObstacleDirection(segment, runner.position + runner.nextStep, 0.1) match {
        case GridDirection.Top => Some(toBottom)
        case GridDirection.Right => Some(toLeft)
        case GridDirection.Bottom => Some(toTop)
        case GridDirection.Left => Some(toRight)
        case _ => throw new UnsupportedOperationException("Unknown direction")
      }).fold(Vec2.zero)((v1: Vec2, v2: Vec2) => v1 + v2)

      val repellingForceFactor = 1.0
      val repellingForce = repellingForceBase * repellingForceFactor

      runner.withAppliedForceConstrainedNoMin(
        repellingForce,
        config.personUnitAcceleration,
        config.cellSize)
    }
  }

  private def getObstacleDirection(line: Line, position: Vec2, minDistance: Double): GridDirection = {
    val hLine: Line = Line(Vec2(position.x - minDistance * 2, position.y), Vec2(position.x + minDistance * 2, position.y))
    val vLine: Line = Line(Vec2(position.x, position.y - minDistance * 2), Vec2(position.x, position.y + minDistance * 2))

    val intersections: Seq[Option[LineIntersection]] = Seq(line.intersect(hLine), line.intersect(vLine))
    val nearestPoint: LineIntersection = intersections.filter(intersection => intersection.nonEmpty)
      .map(intersection => intersection.get)
      .find(intersection => intersection.onLine1 && intersection.onLine2)
      .orNull

    // fixme null ptr exception - probably it was just bad initial agent positioning/config
    if (nearestPoint.pos.x - position.x > 0) {
      GridDirection.Right
    }
    else if (nearestPoint.pos.x - position.x < 0) {
      GridDirection.Left
    }
    else if (nearestPoint.pos.y - position.y > 0) {
      GridDirection.Top
    }
    else {
      GridDirection.Bottom
    }
  }

  private def moveRunnersFromCell(gridCellId: CellId,
                                  cell: ContinuousEnvCell,
                                  neighbourContents: Map[(ContinuousEnvCell, UUID), GridDirection],
                                  state: NeighbourhoodState,
                                  allReachableRunners: Array[Runner],
                                  config: ContinuousEnvConfig): Plans = {
    val plansGroupedByCellId: Map[CellId, Set[Plan]] = cell.runners.toSet
      .map(runner => moveRunner(
        cell.generation,
        runner,
        cell,
        gridCellId,
        allReachableRunners.toSet - runner,
        neighbourContents,
        state,
        config))
      .groupMap {
        case (cellId, _) => cellId
      } {
        case (_, plan) => plan
      }

    val localPlans: Seq[Plan] = plansGroupedByCellId
      .getOrElse(gridCellId, Seq(Plan(StateUpdate(RunnerOccupied(cell.generation + 1, Set())))))
      .toSeq
    val outwardPlans: Map[CellId, Seq[Plan]] = plansGroupedByCellId
      .filterNot {
        case (cellId, _) => cellId.equals(gridCellId)
      }
      .map {
        case (cellId, value) => (cellId, value.toSeq)
      }

    //reportDiagnostics(RunnerChangeCellDiagnostic(gridCellId, outwardPlans))

    new Plans(outwardPlans, localPlans)
  }

  private def moveRunner(generation: Long,
                         runner: Runner,
                         cell: ContinuousEnvCell,
                         cellId: CellId,
                         allReachableRunners: Set[Runner],
                         neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                         state: NeighbourhoodState,
                         config: ContinuousEnvConfig): (CellId, Plan) = {
    if (runner.nextStep.lengthSq > 0) {
      //      val onTheRightSideRunners = filterRunnersBySide(runner, allReachableRunners.toSeq)
      val inflatedRunners = inflateRunners(allReachableRunners.toSeq)

      val moveCompletion = tryGetMaxMoveCompletionOrMin(() => runner.getMaxMoveCompletion(
        inflatedRunners,
        neighbourContents,
        cell,
        config.cellSize))
      //reportDiagnostics(RunnerMoveCompletionDiagnostic(runner, moveCompletion))

      val safeMoveCompletion = moveCompletion.safeRounded
      //reportDiagnostics(RunnerSafeRoundedMoveCompletionDiagnostic(runner, safeMoveCompletion))

      val movedRunner = runner.completeMove(safeMoveCompletion)
      val (normalizedRunner, destinationDirection) = movedRunner.normalizePosition(config.cellSize)
      if (destinationDirection.isEmpty) {
        (cellId, Plan(StateUpdate(RunnerOccupied(generation + 1, Set(normalizedRunner)))))
      }
      else {
        normalizedRunner.path = List.empty
        (getTargetNeighbour(cell.neighbourhood, cell, destinationDirection.get.asInstanceOf[GridDirection], Line(runner.position, movedRunner.position)),
          Plan(StateUpdate(RunnerOccupied(generation + 1, Set(normalizedRunner)))))
      }
      //musi być runner occupied, żeby móc kilkukrotnie przy resolve plans wpisać dodatkowych agentów do tej samej komórki tak żeby się wszyscy zapisali
    } else {
      (cellId, Plan(StateUpdate(RunnerOccupied(generation + 1, Set(runner)))))
    }
  }

  private def getTargetNeighbour(neighbourhood: Neighbourhood,
                                 cell: ContinuousEnvCell,
                                 gridDirection: GridDirection,
                                 runnerStep: Line): GridMultiCellId = {
    if (gridDirection.isDiagonal) {
      return neighbourhood.diagonalNeighbourhood
        .filter {
          case (direction, _) => direction.equals(gridDirection)
        }
        .map {
          case (_, cellId) => cellId
        }
        .headOption.orNull
    }
    getCardinalTargetNeighbour(runnerStep, cell.cardinalSegments)
  }

  private def getCardinalTargetNeighbour(runnerStep: Line,
                                         cardinalNeighbourLineMap: Map[Line, GridMultiCellId]): GridMultiCellId = {
    cardinalNeighbourLineMap
      .map(neighbourEntry => (neighbourEntry._1.intersect(runnerStep), neighbourEntry._2))
      .filter(intersectionEntry => intersectionEntry._1.nonEmpty)
      .filter(intersectionEntry => intersectionEntry._1.get.onLine1 && intersectionEntry._1.get.onLine2)
      .head._2
  }

  private def getTargetCell(cells: Iterable[ContinuousEnvCell],
                            runner: Runner): ContinuousEnvCell = {
    cells.find(cell => (cell.cellOutline.x.doubleValue <= runner.position.x && runner.position.x <= (cell.cellOutline.x + cell.cellOutline.width).doubleValue)
      && (cell.cellOutline.y.doubleValue <= runner.position.y && runner.position.y <= (cell.cellOutline.y + cell.cellOutline.height).doubleValue)).orNull
  }

  private def inflateRunners(runners: Seq[Runner]): Seq[Runner] = runners.map(_.inflate(0.01))

  private def getAllReachableRunners(cell: ContinuousEnvCell,
                                     neighbours: Map[(ContinuousEnvCell, UUID), GridDirection],
                                     config: ContinuousEnvConfig): Array[Runner] = {
    var result: Array[Runner] = cell.runners
    neighbours
      .filter(neighbour => neighbour._1._1.runners.nonEmpty)
      .foreach(neighbour => result ++= neighbour._1._1.runners.map(x => x.withAdjustedPosition(config.cellSize, neighbour._2)))
    result
  }

  private def tryGetMaxMoveCompletionOrMin(moveCompletionComputationAction: () => MoveCompletion): MoveCompletion =
    try {
      moveCompletionComputationAction()
    } catch {
      case e: Exception =>
        e.printStackTrace()
        MoveCompletion.min(tag = "e")
    }

  private def getCardinalNeighbourId(neighbourhood: Neighbourhood,
                                     state: NeighbourhoodState,
                                     neighbour: ContinuousEnvCell,
                                     neighbourDirection: GridDirection): GridMultiCellId = {
    val neighbourSegment: Segment = getSegmentForCardinalNeighbour(state, neighbour, neighbourDirection)
    neighbourhood.cardinalNeighbourhood
      .filter {
        case (direction, _) => direction.equals(neighbourDirection)
      }
      .map {
        case (_, boundary) => boundary.boundaries
      }
      .flatten.toMap
      .filter {
        case (segment, _) => segment.equals(neighbourSegment)
      }
      .map {
        case (_, cellId) => cellId
      }
      .headOption.orNull
  }

  private def getSegmentForCardinalNeighbour(state: NeighbourhoodState,
                                             neighbour: ContinuousEnvCell,
                                             neighbourDirection: GridDirection): Segment = {
    state.cardinalNeighbourhoodState
      .filter {
        case (direction, _) => direction.equals(neighbourDirection)
      }
      .map {
        case (_, boundary) => boundary.boundaryStates
      }
      .flatten.toMap
      .map {
        case (segment, cellState) => (segment, cellState.contents.asInstanceOf[ContinuousEnvCell])
      }
      .filter {
        case (_, continuousEnvCell) => continuousEnvCell.equals(neighbour)
      }
      .map {
        case (segment, _) => segment
      }
      .headOption.orNull
  }
}
