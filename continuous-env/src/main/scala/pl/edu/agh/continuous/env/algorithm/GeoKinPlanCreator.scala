package pl.edu.agh.continuous.env.algorithm

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
        var destination = Line(runner.position, Vec2(runner.position.x + force.x * config.cellSize * math.sqrt(2.0),
          runner.position.y + force.y * config.cellSize * math.sqrt(2.0)))
        destination = limitLineToNeighbourObstacles(destination, neighbourContents, config.cellSize)
        if (cell.graph.isEmpty) {
          runner.path = List(destination.start, destination.end)
        }
        else {
          runner.path = findPath(destination.start, destination.end, cell.graph, config.cellSize).toList
          //TODO ograniczenie ruchu jak agent idzie na skos
          //TODO użycie mapy cardinalSegments do wyznaczania id sąsiada na podstawie tego, jaki segment przekroczy agent
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
        case e: NoSuchElementException => runner.path = findPath(runner.position, runner.path.last, cell.graph, config.cellSize).toList
          target = findNextStep(runner.path, runner.position, cell, neighbourContents, config.cellSize)
      }
      val speed = 20.0 //TODO replace with agent speed
      val movementVector = Line(runner.position, target)
      nextStep = Vec2((movementVector.end.x - movementVector.start.x) / movementVector.length * speed,
        (movementVector.end.y - movementVector.start.y) / movementVector.length * speed)
    }

    val adjustedRunner = runner //TODO acceleration must be increased by cellsize^2
      .withNewPriority()
      .withIncrementedGeneration()
      .withAppliedForceConsideringLastStep(
        nextStep,
        config.personUnitAcceleration,
        config.personMinStepLength,
        config.cellSize)

    reportPossibleFulfillmentDiagnostics(adjustedRunner, config.cellSize)
    adjustedRunner
  }

  private def findNextStep(path: List[Vec2],
                           startingPosition: Vec2,
                           cell: ContinuousEnvCell,
                           neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                           cellSize: Int): Vec2 = {
    path.findLast(vertice => !ObstacleMapping.NeighborContentsExtensions(neighbourContents + ((cell, UUID.randomUUID()) -> null))
      .mapToObstacleLines(cellSize)
      .map(line => (line, line.intersect(Line(startingPosition, vertice))))
      .filter(intersection => intersection._2.nonEmpty)
      .exists(intersection => intersection._2.get.onLine1 && intersection._2.get.onLine2))
      .head
  }

  private def limitLineToNeighbourObstacles(line: Line,
                                            neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                            cellSize: Int): Line = {
    var nearestCollisionPoint: Vec2 = Vec2.zero
    try {
      nearestCollisionPoint = ObstacleMapping.NeighborContentsExtensions(neighbourContents)
        .mapToObstacleLines(cellSize)
        .map(line => line.intersect(line))
        .filter(intersection => intersection.nonEmpty)
        .filter(intersection => intersection.get.onLine1 && intersection.get.onLine2)
        .map(intersection => intersection.get.pos)
        .minBy(intersectionPoint => Line(line.start, intersectionPoint).length)
    }
    catch {
      case e: UnsupportedOperationException => return line
    }
    val newLineLength: Double = Line(line.start, nearestCollisionPoint).length
    val normalized: Vec2 = Vec2((nearestCollisionPoint.x - line.start.x) / newLineLength,
      (nearestCollisionPoint.y - line.start.y) / newLineLength)
    Line(line.start, Vec2(line.start.x + normalized.x * newLineLength * 0.99,
      line.start.y + normalized.y * newLineLength * 0.99))
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
    val toBottom = Vec2(1, 0)
    val toTop = -toBottom
    val toLeft = Vec2(0, 1)
    val toRight = -toLeft

    val cells: Map[(ContinuousEnvCell, UUID), Direction] = neighbourContents + ((currentCell, UUID.randomUUID()) -> null)
    val obstacleSegments: Iterable[Line] = ObstacleMapping.NeighborContentsExtensions(cells)
      .mapToObstacleLines(config.cellSize)
      .filter(line => line.segmentDistance(runner.position + runner.nextStep) < 0.1)
    if (obstacleSegments.isEmpty) {
      runner
    }
    else {
      val repellingForceBase = obstacleSegments.flatMap(segment => getObstacleDirection(segment, runner.nextStep, 0.1) match {
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

    // fixme null ptr exception - maybe it was just bad initial agent positioning/config
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
        (getTargetNeighbour(neighbourContents, cell.neighbourhood, state, destinationDirection.get.asInstanceOf[GridDirection], normalizedRunner),
          Plan(StateUpdate(RunnerOccupied(generation + 1, Set(normalizedRunner)))))
      }
      //musi być runner occupied, żeby móc kilkukrotnie przy resolve plans wpisać dodatkowych agentów do tej samej komórki tak żeby się wszyscy zapisali
    } else {
      (cellId, Plan(StateUpdate(RunnerOccupied(generation + 1, Set(runner)))))
    }
  }

  private def getTargetNeighbour(neighbourContents: Map[(ContinuousEnvCell, UUID), Direction],
                                 neighbourhood: Neighbourhood,
                                 state: NeighbourhoodState,
                                 gridDirection: GridDirection,
                                 movedRunner: Runner): GridMultiCellId = {
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
    val potentialTargets: Iterable[ContinuousEnvCell] = neighbourContents
      .filter {
        case (_, direction) => direction.equals(gridDirection)
      }
      .map {
        case ((continuousEnvCell, _), _) => continuousEnvCell
      }
    val target: ContinuousEnvCell = getTargetCell(potentialTargets, movedRunner)
    getCardinalNeighbourId(neighbourhood, state, target, gridDirection)
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
