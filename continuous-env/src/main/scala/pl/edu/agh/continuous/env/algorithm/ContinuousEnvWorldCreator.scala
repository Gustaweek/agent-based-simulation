package pl.edu.agh.continuous.env.algorithm

import org.locationtech.jts.geom.{Coordinate, GeometryFactory}
import org.locationtech.jts.operation.buffer.BufferParameters
import org.slf4j.Logger
import pl.edu.agh.continuous.env.common.geometry.Vec2
import pl.edu.agh.continuous.env.config.ContinuousEnvConfig
import pl.edu.agh.continuous.env.model.{ContinuousEnvCell, Runner}
import pl.edu.agh.continuous.env.model.continuous.{Being, BeingMetadata, CellOutline, Obstacle}
import pl.edu.agh.xinuk.algorithm.WorldCreator
import pl.edu.agh.xinuk.model.continuous.{Boundary, GridMultiCellId, Neighbourhood, Segment}
import pl.edu.agh.xinuk.model.grid.GridDirection.{Bottom, BottomLeft, BottomRight, Left, Right, Top, TopLeft, TopRight}
import pl.edu.agh.xinuk.model.{CellState, Signal, WorldBuilder}
import pl.edu.agh.xinuk.model.grid.{GridCellId, GridDirection, GridWorldBuilder}

import java.awt.geom.Area
import java.awt.{Color, Polygon}
import java.util.UUID
import scala.collection.mutable
import scala.collection.mutable.{Map => MutableMap}
import scala.swing.Rectangle
import scala.util.Random


object ContinuousEnvWorldCreator extends WorldCreator[ContinuousEnvConfig] {

  private val random = new Random(System.nanoTime())
  var logger: Logger = _

  override def prepareWorld()(implicit config: ContinuousEnvConfig): WorldBuilder = {
    val worldBuilder = GridWorldBuilder().withGridConnections()

    val multiCellIdMap: MutableMap[GridCellId, Int] = MutableMap.empty
    val cellOutlineMap: MutableMap[GridMultiCellId, CellOutline] = MutableMap.empty.withDefault(_ => CellOutline.default())
    val cellQueue: mutable.Queue[GridMultiCellId] = mutable.Queue.empty
    val finalCellQueue: mutable.Queue[GridMultiCellId] = mutable.Queue.empty

    for {
      x <- 0 until config.worldWidth
      y <- 0 until config.worldHeight
    } {
      cellQueue.enqueue(GridMultiCellId(x, y, 0))
      multiCellIdMap(GridCellId(x, y)) = 0
    }

    val obstacles = bufferObstacles(config.obstacles)

    while (cellQueue.nonEmpty) {
      val gridMultiCellId = cellQueue.dequeue()
      val x = gridMultiCellId.x
      val y = gridMultiCellId.y
      val continuousEnvCell: ContinuousEnvCell = if (x == 60 && y == 80) {
        ContinuousEnvCell(config.initialSignal)
      } else {
        ContinuousEnvCell(Signal.zero)
      }

      continuousEnvCell.neighbourhood = worldBuilder.getExistingNeighbourhood(gridMultiCellId)
      continuousEnvCell.cellOutline = cellOutlineMap(gridMultiCellId)

      val overlappingObstacles = getOverlappingObstacles(continuousEnvCell, obstacles, x, y)

      if (overlappingObstacles.nonEmpty) {
        var obstaclesGroups: Array[Array[Obstacle]] = Array()
        var cellDivided = false

        for (obstacle <- overlappingObstacles) {
          var obstaclesToMerge = Array(obstacle)
          var newObstaclesGroups: Array[Array[Obstacle]] = Array()
          for (existingObstacleGroup <- obstaclesGroups) {
            if (overlapsWithAny(obstacle, existingObstacleGroup)) {
              obstaclesToMerge = obstaclesToMerge ++ existingObstacleGroup
            } else {
              newObstaclesGroups = newObstaclesGroups :+ existingObstacleGroup
            }
          }
          newObstaclesGroups = newObstaclesGroups :+ obstaclesToMerge
          obstaclesGroups = newObstaclesGroups

          if (isObstaclesGroupDividingCell(continuousEnvCell.cellOutline, obstaclesToMerge, x, y)) {
            val newCells = divideCells(continuousEnvCell, obstaclesToMerge, x, y)
              .filter(cell => !hasEmptyNeighbourhood(cell))
              .filter(cell => !hasSameNeighbourhood(cell, continuousEnvCell))

            if (newCells.nonEmpty) {
              cellDivided = true

              val currentId = multiCellIdMap(GridCellId(x, y))
              val currentGridMultiCellId = GridMultiCellId(x, y, currentId)
              val newCellNeighbourhoodMap: MutableMap[GridMultiCellId, Neighbourhood] = MutableMap.empty
              for (i <- newCells.indices) {
                val nextId = currentId + i + 1
                val newCellGridMultiCellId = GridMultiCellId(x, y, nextId)
                newCellNeighbourhoodMap(newCellGridMultiCellId) = newCells(i).neighbourhood
                cellOutlineMap(newCellGridMultiCellId) = newCells(i).cellOutline
                cellQueue.enqueue(newCellGridMultiCellId)
              }
              worldBuilder.updateNeighbourhoodAfterDividingCell(currentGridMultiCellId, Map.from(newCellNeighbourhoodMap))
              multiCellIdMap(GridCellId(x, y)) = currentId + newCells.length
            }
          }
        }

        if (!cellDivided) {
          continuousEnvCell.obstacles = mergeToLocalObstacles(obstaclesGroups, x, y)
          continuousEnvCell.neighbourhood = updateNeighbourhoodWithObstacles(continuousEnvCell)

          worldBuilder.updateNeighbourhood(gridMultiCellId, continuousEnvCell.neighbourhood)

          finalCellQueue.enqueue(gridMultiCellId)
        }
      } else {
        finalCellQueue.enqueue(gridMultiCellId)
      }

      worldBuilder(gridMultiCellId) = CellState(continuousEnvCell)
    }

    while (finalCellQueue.nonEmpty) {
      val gridMultiCellId = finalCellQueue.dequeue()
      val continuousEnvCell: ContinuousEnvCell = worldBuilder(gridMultiCellId).state.contents.asInstanceOf[ContinuousEnvCell]

      if (gridMultiCellId.x == 50 && gridMultiCellId.y == 45) {
        //val runner1: Runner = Runner.createNew(Vec2(50, 50), 10)
        val runner2: Runner = Runner.createNew(Vec2(65, 35), 30)
        var guiMapping: Map[UUID, (Double, Double)] = Map.empty
        //guiMapping += (runner1.id -> (50.0, 50.0))
        guiMapping += (runner2.id -> (65.0, 35.0))
        continuousEnvCell.runners = Seq(runner2).toArray
        continuousEnvCell.coordinates = guiMapping
      }

      val boundaryObstacles = getBoundaryObstacles(continuousEnvCell)
      val allObstacles = boundaryObstacles ++ continuousEnvCell.obstacles

      if (boundaryObstacles.nonEmpty) {
        var obstaclesGroups: Array[Array[Obstacle]] = Array()

        for (obstacle <- allObstacles) {
          var obstaclesToMerge = Array(obstacle)
          var newObstaclesGroups: Array[Array[Obstacle]] = Array()
          for (existingObstacleGroup <- obstaclesGroups) {
            if (overlapsWithAny(obstacle, existingObstacleGroup)) {
              obstaclesToMerge = obstaclesToMerge ++ existingObstacleGroup
            } else {
              newObstaclesGroups = newObstaclesGroups :+ existingObstacleGroup
            }
          }
          newObstaclesGroups = newObstaclesGroups :+ obstaclesToMerge
          obstaclesGroups = newObstaclesGroups
        }

        val obstaclesInCell = mergeToObstacles(obstaclesGroups)
        continuousEnvCell.obstacles = obstaclesInCell
      }

      worldBuilder(gridMultiCellId) = CellState(continuousEnvCell)
    }

    worldBuilder
  }

  private def getBoundaryObstacles(cell: ContinuousEnvCell)
                                  (implicit config: ContinuousEnvConfig): Array[Obstacle] = {
    cell.neighbourhood.cardinalNeighbourhood
      .map { case (direction, boundary) => getBoundaryObstaclesInDirection(direction, boundary, cell) }
      .flatten
      .toArray
  }

  private def getBoundaryObstaclesInDirection(direction: GridDirection, boundary: Boundary, cell: ContinuousEnvCell)
                                             (implicit config: ContinuousEnvConfig): Array[Obstacle] = {
    getNonBoundarySegments(boundary, direction, cell)
      .map(segment => getBoundaryObstacleInDirection(direction, segment, cell))
  }

  private def getNonBoundarySegments(boundary: Boundary, direction: GridDirection, cell: ContinuousEnvCell)
                                    (implicit config: ContinuousEnvConfig): Array[Segment] = {
    var nonBoundarySegments: Array[Segment] = if (direction == Top || direction == Bottom) {
      Array(Segment(cell.cellOutline.x, cell.cellOutline.x + cell.cellOutline.width))
    } else {
      Array(Segment(cell.cellOutline.y, cell.cellOutline.y + cell.cellOutline.height))
    }

    for (boundarySegment <- boundary.boundaries.keys) {
      var newNonBoundarySegments: Array[Segment] = Array()
      for (nonBoundarySegment <- nonBoundarySegments) {
        if (nonBoundarySegment.a < boundarySegment.a && nonBoundarySegment.b > boundarySegment.b) {
          val leftSegment = Segment(nonBoundarySegment.a, boundarySegment.a)
          val rightSegment = Segment(boundarySegment.b, nonBoundarySegment.b)
          newNonBoundarySegments = newNonBoundarySegments :+ leftSegment :+ rightSegment
        } else if (nonBoundarySegment.a < boundarySegment.a && nonBoundarySegment.b <= boundarySegment.b) {
          val leftSegment = Segment(nonBoundarySegment.a, boundarySegment.a)
          newNonBoundarySegments = newNonBoundarySegments :+ leftSegment
        } else if (nonBoundarySegment.a >= boundarySegment.a && nonBoundarySegment.b > boundarySegment.b) {
          val rightSegment = Segment(boundarySegment.b, nonBoundarySegment.b)
          newNonBoundarySegments = newNonBoundarySegments :+ rightSegment
        } else if (nonBoundarySegment.a >= boundarySegment.a && nonBoundarySegment.b <= boundarySegment.b) {
          // nothing
        }
      }
      nonBoundarySegments = newNonBoundarySegments
    }

    nonBoundarySegments
  }

  private def getBoundaryObstacleInDirection(direction: GridDirection, segment: Segment, cell: ContinuousEnvCell)
                                            (implicit config: ContinuousEnvConfig): Obstacle = {
    direction match {
      case Top =>
        val xs = Array(segment.a, segment.a, segment.b, segment.b)
        val ys = Array(0, 1, 1, 0).map(y => y + cell.cellOutline.y + cell.cellOutline.height)

        Obstacle(xs, ys, 4)
      case Right =>
        val xs = Array(0, 1, 1, 0).map(x => x + cell.cellOutline.x + cell.cellOutline.width)
        val ys = Array(segment.b, segment.b, segment.a, segment.a)

        Obstacle(xs, ys, 4)
      case Bottom =>
        val xs = Array(segment.a, segment.b, segment.b, segment.a)
        val ys = Array(0, 0, -1, -1).map(y => y + cell.cellOutline.y)

        Obstacle(xs, ys, 4)
      case Left =>
        val xs = Array(0, -1, -1, 0).map(x => x + cell.cellOutline.x)
        val ys = Array(segment.a, segment.a, segment.b, segment.b)

        Obstacle(xs, ys, 4)
      case _ => null
    }
  }

  private def bufferObstacles(obstacles: List[Obstacle])
                             (implicit continuousEnvConfig: ContinuousEnvConfig): List[Obstacle] = {
    obstacles
      .map(obstacle => bufferObstacle(obstacle))
  }

  private def bufferObstacle(obstacle: Obstacle)
                            (implicit config: ContinuousEnvConfig): Obstacle = {
    var jtsCoordinates: Array[Coordinate] = Range(0, obstacle.points)
      .map(i => new Coordinate(obstacle.xs(i), obstacle.ys(i)))
      .toArray
    jtsCoordinates = jtsCoordinates :+ new Coordinate(obstacle.xs(0), obstacle.ys(0))

    val geometryFactory = new GeometryFactory()
    val shell = geometryFactory.createPolygon(jtsCoordinates)
    val bufferedPolygon = shell.buffer(config.beingRadius, BufferParameters.CAP_FLAT)

    var newXs: Array[Int] = Array()
    var newYs: Array[Int] = Array()

    bufferedPolygon
      .getCoordinates
      .take(bufferedPolygon.getCoordinates.length - 1)
      .foreach(coordinate => {
        newXs = newXs :+ coordinate.x.intValue
        newYs = newYs :+ coordinate.y.intValue
      })

    Obstacle(newXs, newYs, newXs.length)
  }

  private def getOverlappingObstacles(continuousEnvCell: ContinuousEnvCell, obstacles: List[Obstacle], x: Int, y: Int)
                                     (implicit config: ContinuousEnvConfig): List[Obstacle] = {
    val cellOutline = continuousEnvCell.cellOutline
    val xScale = y
    val yScale = config.worldWidth - x - 1

    val cellOutlineArea = new Area(new Rectangle(
      xScale * config.cellSize + cellOutline.x.intValue, yScale * config.cellSize + cellOutline.y.intValue,
      cellOutline.width.intValue, cellOutline.height.intValue))
    val obstaclesAreas = obstacles
      .map(obstacle => new Area(new Polygon(obstacle.xs, obstacle.ys, obstacle.points)))
    obstaclesAreas
      .foreach(obstacleArea => obstacleArea.intersect(cellOutlineArea))
    val overlappingObstaclesAreas = obstaclesAreas
      .filter(obstacleArea => !obstacleArea.isEmpty)
      .map(overlappingObstacleArea => toObstacle(overlappingObstacleArea))
      .map(obstacle => fixArtifactsInObstacle(obstacle, cellOutline, x, y))

    overlappingObstaclesAreas
  }

  private def fixArtifactsInObstacle(obstacle: Obstacle, cellOutline: CellOutline, x: Int, y: Int)
                                    (implicit config: ContinuousEnvConfig): Obstacle = {
    val xScale = y
    val yScale = config.worldWidth - x - 1
    val xBoundaries: Array[Int] = Array(
      xScale * config.cellSize + cellOutline.x.intValue,
      xScale * config.cellSize + cellOutline.x.intValue + cellOutline.width.intValue,
    )
    val yBoundaries: Array[Int] = Array(
      yScale * config.cellSize + cellOutline.y.intValue,
      yScale * config.cellSize + cellOutline.y.intValue + cellOutline.height.intValue
    )

    var newXs: Array[Int] = Array()
    var newYs: Array[Int] = Array()

    for (i <- 0 until obstacle.points) {
      var newX = obstacle.xs(i)
      var newY = obstacle.ys(i)

      if (math.abs(newX - xBoundaries(0)) == 1) {
        newX = xBoundaries(0)
      } else if (math.abs(newX - xBoundaries(1)) == 1) {
        newX = xBoundaries(1)
      }

      if (math.abs(newY - yBoundaries(0)) == 1) {
        newY = yBoundaries(0)
      } else if (math.abs(newY - yBoundaries(1)) == 1) {
        newY = yBoundaries(1)
      }

      newXs = newXs :+ newX
      newYs = newYs :+ newY
    }

    var uniquePoints: Array[(Int, Int)] = Array()
    for (i <- 0 until obstacle.points) {
      uniquePoints = uniquePoints :+ (newXs(i), newYs(i))
    }
    uniquePoints = uniquePoints.distinct

    newXs = Array()
    newYs = Array()

    uniquePoints.foreach(point => {
        newXs = newXs :+ point._1
        newYs = newYs :+ point._2
      }
    )

    Obstacle(newXs, newYs, newXs.length)
  }

  private def toObstacle(overlappingObstacleArea: Area): Obstacle = {
    val pathIterator = overlappingObstacleArea.getPathIterator(null)
    val coords = Array(0d, 0d, 0d, 0d, 0d, 0d)
    var xs: Array[Int] = Array()
    var ys: Array[Int] = Array()
    while (!pathIterator.isDone) {
      pathIterator.currentSegment(coords)
      xs = xs :+ coords(0).intValue
      ys = ys :+ coords(1).intValue
      pathIterator.next()
    }

    xs = xs.dropRight(1)
    ys = ys.dropRight(1)

    Obstacle(xs, ys, xs.length)
  }

  private def overlapsWithAny(obstacle: Obstacle, obstaclesGroup: Array[Obstacle]): Boolean = {
    obstaclesGroup.exists(existingObstacle => overlaps(obstacle, existingObstacle))
  }

  private def overlaps(obstacle: Obstacle, existingObstacle: Obstacle): Boolean = {
    anyPointLiesIn(obstacle, existingObstacle) ||
      anyPointLiesIn(existingObstacle, obstacle) ||
      anyPointLiesOn(obstacle, existingObstacle) ||
      anyPointLiesOn(existingObstacle, obstacle)
  }

  private def anyPointLiesIn(first: Obstacle, second: Obstacle): Boolean = {
    var result = false
    val poly = new Polygon(first.xs, first.ys, first.points)
    for (i <- 0 until second.points) {
      if (poly.contains(second.xs(i), second.ys(i))) {
        result = true
      }
    }

    result
  }

  private def anyPointLiesOn(first: Obstacle, second: Obstacle): Boolean = {
    var result = false
    for (i <- 0 until first.points) {
      val cX = first.xs(i)
      val cY = first.ys(i)
      for (j <- 0 until second.points) {
        val aX = second.xs(j)
        val aY = second.ys(j)
        val bX = second.xs((j + 1) % second.points)
        val bY = second.ys((j + 1) % second.points)

        if (squaredDistance(aX, aY, cX, cY) + squaredDistance(cX, cY, bX, bY) == squaredDistance(aX, aY, bX, bY)) {
          result = true
        }
      }
    }

    result
  }

  private def squaredDistance(x1: Int, y1: Int, x2: Int, y2: Int): Int = {
    (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
  }

  private def isObstaclesGroupDividingCell(cellOutline: CellOutline, obstaclesGroup: Array[Obstacle], cellX: Int, cellY: Int)
                                          (implicit config: ContinuousEnvConfig): Boolean = {
    var obstacle = mergeToObstacle(obstaclesGroup)
    obstacle = addDummyPointsBetweenPointsLyingOnEdges(cellOutline, obstacle, cellX, cellY)
    val flags = pointsOnCellOutline(cellOutline, obstacle, cellX, cellY)
    val flips = countFlips(flags)

    flips > 2
  }

  private def mergeToObstacle(obstaclesGroup: Array[Obstacle]): Obstacle = {
    val mergedObstacleArea = new Area()
    for (obstacle <- obstaclesGroup) {
      val obstaclePoly = new Polygon(obstacle.xs, obstacle.ys, obstacle.points)
      mergedObstacleArea.add(new Area(obstaclePoly))
    }

    toObstacle(mergedObstacleArea)
  }

  private def addDummyPointsBetweenPointsLyingOnEdges(cellOutline: CellOutline, obstacle: Obstacle, cellX: Int, cellY: Int)
                                                     (implicit config: ContinuousEnvConfig): Obstacle = {
    var newXs: Array[Int] = Array()
    var newYs: Array[Int] = Array()

    for (i <- 0 until obstacle.points) {
      val currentX = obstacle.xs(i)
      val currentY = obstacle.ys(i)
      val nextX = obstacle.xs((i + 1) % obstacle.points)
      val nextY = obstacle.ys((i + 1) % obstacle.points)
      if (!isLyingInsideCellOutline(cellOutline, currentX, currentY, cellX, cellY)
          && !isLyingInsideCellOutline(cellOutline, nextX, nextY, cellX, cellY)) {
        val dummyX = (currentX + nextX) / 2
        val dummyY = (currentY + nextY) / 2

        newXs = newXs :+ currentX :+ dummyX
        newYs = newYs :+ currentY :+ dummyY
      } else {
        newXs = newXs :+ currentX
        newYs = newYs :+ currentY
      }
    }

    Obstacle(newXs, newYs, newXs.length)
  }

  private def isLyingInsideCellOutline(cellOutline: CellOutline, x: Int, y: Int, cellX: Int, cellY: Int)
                                      (implicit config: ContinuousEnvConfig): Boolean = {
    val xScale = cellY
    val yScale = config.worldWidth - cellX - 1

    val localX = x - xScale * config.cellSize
    val localY = y - yScale * config.cellSize
    localX > cellOutline.x && localX < cellOutline.x + cellOutline.width && localY > cellOutline.y && localY < cellOutline.y + cellOutline.height
  }

  private def pointsOnCellOutline(cellOutline: CellOutline, obstacle: Obstacle, cellX: Int, cellY: Int)
                                 (implicit config: ContinuousEnvConfig): Array[Boolean] = {
    var flags: Array[Boolean] = Array()
    for (i <- 0 until obstacle.points) {
      val currentX = obstacle.xs(i)
      val currentY = obstacle.ys(i)
      flags = flags :+ !isLyingInsideCellOutline(cellOutline, currentX, currentY, cellX, cellY)
    }

    flags
  }

  private def countFlips(flags: Array[Boolean]): Int = {
    var flips = 0
    for (i <- flags.indices) {
      if (flags(i) != flags((i + 1) % flags.length)) {
        flips = flips + 1
      }
    }

    flips
  }

  private def divideCells(cell: ContinuousEnvCell, obstaclesGroup: Array[Obstacle], cellX: Int, cellY: Int)
                         (implicit config: ContinuousEnvConfig): Array[ContinuousEnvCell] = {
    var obstacle = mergeToObstacle(obstaclesGroup)
    obstacle = addDummyPointsBetweenPointsLyingOnEdges(cell.cellOutline, obstacle, cellX, cellY)
    val flags = pointsOnCellOutline(cell.cellOutline, obstacle, cellX, cellY)

    val newCellBoundaries: Array[Array[(Int, Int)]] = extractNewCellBoundaries(cell.cellOutline, obstacle, flags, cellX, cellY)

    val newCells = newCellBoundaries
      .map(cellBoundary => createNewCell(cell, cellBoundary))

    newCells
  }

  private def extractNewCellBoundaries(cellOutline: CellOutline, obstacle: Obstacle, flags: Array[Boolean], cellX: Int, cellY: Int)
                                      (implicit config: ContinuousEnvConfig): Array[Array[(Int, Int)]] = {
    val localObstaclePoints: Array[(Int, Int)] = toLocalObstaclePoints(obstacle, cellX, cellY)
    var newCellBoundaries: Array[Array[(Int, Int)]] = Array()
    val newCellsNum = countFlips(flags) / 2

    var start = 0
    for (i <- 0 until newCellsNum) {
      val nextTrueBeforeFalse = findNextTrueBeforeFalse(flags, start)
      val nextTrue = findNextTrue(flags, nextTrueBeforeFalse + 1)

      var cellSplit: Array[(Int, Int)] = Array()
      for (j <- nextTrueBeforeFalse to nextTrue) {
        cellSplit = cellSplit :+ localObstaclePoints(j % localObstaclePoints.length)
      }

      cellSplit = addCellOutlineVerticesToSplit(cellOutline, cellSplit)

      newCellBoundaries = newCellBoundaries :+ cellSplit
      start = nextTrue + 1
    }

    newCellBoundaries
  }

  private def toLocalObstaclePoints(obstacle: Obstacle, cellX: Int, cellY: Int)
                                   (implicit config: ContinuousEnvConfig): Array[(Int, Int)] = {
    var localObstaclePoints: Array[(Int, Int)] = Array()
    for (i <- 0 until obstacle.points) {
      val xScale = cellY
      val yScale = config.worldWidth - cellX - 1

      val localX = obstacle.xs(i) - xScale * config.cellSize
      val localY = obstacle.ys(i) - yScale * config.cellSize

      localObstaclePoints = localObstaclePoints :+ (localX, localY)
    }

    localObstaclePoints
  }

  private def toLocalObstacle(obstacle: Obstacle, cellX: Int, cellY: Int)
                             (implicit config: ContinuousEnvConfig): Obstacle = {
    var localObstaclePointsX: Array[Int] = Array()
    var localObstaclePointsY: Array[Int] = Array()
    for (i <- 0 until obstacle.points) {
      val xScale = cellY
      val yScale = config.worldWidth - cellX - 1

      val localX = obstacle.xs(i) - xScale * config.cellSize
      val localY = obstacle.ys(i) - yScale * config.cellSize

      localObstaclePointsX = localObstaclePointsX :+ localX
      localObstaclePointsY = localObstaclePointsY :+ localY
    }

    Obstacle(localObstaclePointsX, localObstaclePointsY, localObstaclePointsX.length)
  }

  private def findNextTrueBeforeFalse(flags: Array[Boolean], start: Int): Int = {
    var result = -1
    var found = false
    for (i <- 0 to flags.length if !found) {
      if (flags((start + i) % flags.length) && !flags((start + i + 1) % flags.length)) {
        result = start + i
        found = true
      }
    }

    result
  }

  private def findNextTrue(flags: Array[Boolean], start: Int): Int = {
    var result = -1
    var found = false
    for (i <- 0 to flags.length if !found) {
      if (flags((start + i) % flags.length)) {
        result =  start + i
        found = true
      }
    }

    result
  }

  private def addCellOutlineVerticesToSplit(cellOutline: CellOutline, cellSplit: Array[(Int, Int)]): Array[(Int, Int)] = {
    val firstBoundaryNum = getBoundaryNumForPoint(cellOutline, cellSplit(0))
    var lastBoundaryNum = getBoundaryNumForPoint(cellOutline, cellSplit(cellSplit.length - 1))

    val cellOutlineVertices: Array[(Int, Int)] = Array((cellOutline.x, cellOutline.y + cellOutline.height),
                                                       (cellOutline.x + cellOutline.width, cellOutline.y + cellOutline.height),
                                                       (cellOutline.x + cellOutline.width, cellOutline.y),
                                                       (cellOutline.x, cellOutline.y))

    var verticesToAdd: Array[(Int, Int)] = Array()
    if (lastBoundaryNum < firstBoundaryNum) {
      lastBoundaryNum = lastBoundaryNum + 4
    }

    try {
      for (i <- firstBoundaryNum until lastBoundaryNum) {
        if (cellOutlineVertices(i % 4) != cellSplit(0)) {
          verticesToAdd = verticesToAdd :+ cellOutlineVertices(i % 4)
        }
      }
    } catch {
      case e: Exception =>
        println("oops")
    }

    cellSplit.reverse :++ verticesToAdd
  }

  private def getBoundaryNumForPoint(cellOutline: CellOutline, point: (Int, Int)): Int = {
    val x = point._1
    val y = point._2

    var boundaryNum = -1
    if (x == cellOutline.x && y > cellOutline.y && y <= cellOutline.y + cellOutline.height) {
      boundaryNum = 0
    } else if (x > cellOutline.x && x <= cellOutline.x + cellOutline.width && y == cellOutline.y + cellOutline.height) {
      boundaryNum = 1
    } else if (x == cellOutline.x + cellOutline.width && y >= cellOutline.y && y < cellOutline.y + cellOutline.height) {
      boundaryNum = 2
    } else if (x <= cellOutline.x + cellOutline.width && x > cellOutline.x && y == cellOutline.y) {
      boundaryNum = 3
    }

    boundaryNum
  }

  private def createNewCell(existingCell: ContinuousEnvCell, newCellBoundary: Array[(Int, Int)])
                           (implicit config:ContinuousEnvConfig): ContinuousEnvCell = {
    val existingNeighbourhood = existingCell.neighbourhood
    var newCellNeighbourhood = Neighbourhood.empty()
    val cardinalNeighbourhood: MutableMap[GridDirection, Boundary] = MutableMap.from(newCellNeighbourhood.cardinalNeighbourhood)
    val diagonalNeighbourhood: MutableMap[GridDirection, GridMultiCellId] = MutableMap.from(newCellNeighbourhood.diagonalNeighbourhood)

    for (i <- newCellBoundary.indices) {
      val start = newCellBoundary(i)
      val end = newCellBoundary((i + 1) % newCellBoundary.length)
      if (isVertical(start, end)) { // can be left or right
        var segment = Segment(start._2, end._2)
        if (isLeft(existingCell, start)) {
          cardinalNeighbourhood(Left) = getNewCardinalBoundary(existingNeighbourhood, Left, segment)
        } else if (isRight(existingCell, start)) {
          segment = Segment(segment.b, segment.a) // need to mirror in case it's right boundary
          cardinalNeighbourhood(Right) = getNewCardinalBoundary(existingNeighbourhood, Right, segment)
        }
      } else if (isHorizontal(start, end)) { // can be top or bottom
        var segment = Segment(start._1, end._1)
        if (isTop(existingCell, start)) {
          cardinalNeighbourhood(Top) = getNewCardinalBoundary(existingNeighbourhood, Top, segment)
        } else if (isBottom(existingCell, start)) {
          segment = Segment(segment.b, segment.a) // need to mirror in case it's bottom boundary
          cardinalNeighbourhood(Bottom) = getNewCardinalBoundary(existingNeighbourhood, Bottom, segment)
        }
      }
    }

    for (i <- newCellBoundary.indices) {
      val point = newCellBoundary(i)
      if (isTopLeft(existingCell, point)) {
        diagonalNeighbourhood(TopLeft) = existingNeighbourhood.diagonalNeighbourhood(TopLeft)
      } else if (isTopRight(existingCell, point)) {
        diagonalNeighbourhood(TopRight) = existingNeighbourhood.diagonalNeighbourhood(TopRight)
      } else if (isBottomRight(existingCell, point)) {
        diagonalNeighbourhood(BottomRight) = existingNeighbourhood.diagonalNeighbourhood(BottomRight)
      } else if (isBottomLeft(existingCell, point)) {
        diagonalNeighbourhood(BottomLeft) = existingNeighbourhood.diagonalNeighbourhood(BottomLeft)
      }
    }

    newCellNeighbourhood = Neighbourhood(Map.from(cardinalNeighbourhood), Map.from(diagonalNeighbourhood))
    val newCell = ContinuousEnvCell(existingCell.initialSignal)
    newCell.cellOutline = getNewCellOutline(newCellBoundary)
    newCell.neighbourhood = newCellNeighbourhood

    newCell
  }

  private def isVertical(start: (Int, Int), end: (Int, Int)): Boolean = {
    start._1 == end._1
  }

  private def isHorizontal(start: (Int, Int), end: (Int, Int)): Boolean = {
    start._2 == end._2
  }

  private def isLeft(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.x == point._1
  }

  private def isRight(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.x + continuousEnvCell.cellOutline.width == point._1
  }

  private def isTop(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.y + continuousEnvCell.cellOutline.height == point._2
  }

  private def isBottom(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    continuousEnvCell.cellOutline.y == point._2
  }

  private def isTopLeft(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x == point._1 && cellOutline.y + cellOutline.height == point._2
  }

  private def isTopRight(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x + cellOutline.width == point._1 && cellOutline.y + cellOutline.height == point._2
  }

  private def isBottomRight(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x + cellOutline.width == point._1 && cellOutline.y == point._2
  }

  private def isBottomLeft(continuousEnvCell: ContinuousEnvCell, point: (Int, Int)): Boolean = {
    val cellOutline = continuousEnvCell.cellOutline
    cellOutline.x == point._1 && cellOutline.y == point._2
  }

  private def getNewCellOutline(boundary: Array[(Int, Int)]): CellOutline = {
    val minX = boundary.map(point => point._1).min
    val maxX = boundary.map(point => point._1).max
    val minY = boundary.map(point => point._2).min
    val maxY = boundary.map(point => point._2).max

    CellOutline(minX, minY, maxX - minX, maxY - minY)
  }

  private def getNewCardinalBoundary(existingNeighbourhood: Neighbourhood, cardinalDirection: GridDirection, segment: Segment): Boundary = {
    val boundaryMap = existingNeighbourhood
      .cardinalNeighbourhood(cardinalDirection)
      .boundaries
      .map { case (boundarySegment, gridMultiCellId) => (getCommonSegment(segment, boundarySegment), gridMultiCellId) }
      .filter { case (commonSegment, _) => commonSegment != null }

    Boundary(boundaryMap)
  }

  private def getCommonSegment(segment: Segment, boundarySegment: Segment): Segment = {
    val a = scala.math.max(segment.a, boundarySegment.a)
    val b = scala.math.min(segment.b, boundarySegment.b)

    if (a > b) {
      null
    } else {
      Segment(a, b)
    }
  }

  private def hasEmptyNeighbourhood(continuousEnvCell: ContinuousEnvCell): Boolean = {
    val emptyDiagonalNeighbourhood = continuousEnvCell
      .neighbourhood
      .diagonalNeighbourhood
      .forall { case (_, gridMultiCellId) => gridMultiCellId == null }
    val emptyCardinalNeighbourhood = continuousEnvCell
      .neighbourhood
      .cardinalNeighbourhood
      .forall { case (_, boundary) => boundary.boundaries == Map.empty }

    emptyDiagonalNeighbourhood && emptyCardinalNeighbourhood
  }

  private def hasSameNeighbourhood(cell: ContinuousEnvCell, other: ContinuousEnvCell): Boolean = {
    val hasSameDiagonalNeighbourhood = cell
      .neighbourhood
      .diagonalNeighbourhood
      .forall { case (direction, gridMultiCellId) => gridMultiCellId == other.neighbourhood.diagonalNeighbourhood(direction) }
    val hasSameCardinalNeighbourhood = cell
      .neighbourhood
      .cardinalNeighbourhood
      .forall { case (direction, boundary) => boundary.boundaries == other.neighbourhood.cardinalNeighbourhood(direction).boundaries }

    hasSameDiagonalNeighbourhood && hasSameCardinalNeighbourhood
  }

  private def mergeToLocalObstacles(obstaclesGroups: Array[Array[Obstacle]], cellX: Int, cellY: Int)
                                   (implicit config: ContinuousEnvConfig): Array[Obstacle] = {
    mergeToObstacles(obstaclesGroups)
      .map(obstacle => toLocalObstacle(obstacle, cellX, cellY))
  }

  private def mergeToObstacles(obstaclesGroups: Array[Array[Obstacle]]): Array[Obstacle] = {
    obstaclesGroups
      .map(group => mergeToObstacle(group))
  }

  private def updateNeighbourhoodWithObstacles(continuousEnvCell: ContinuousEnvCell): Neighbourhood = {
    val cardinalNeighbourhood = MutableMap.from(continuousEnvCell.neighbourhood.cardinalNeighbourhood)
    val diagonalNeighbourhood = MutableMap.from(continuousEnvCell.neighbourhood.diagonalNeighbourhood)

    for (obstacle <- continuousEnvCell.obstacles) {
      for (i <- 0 until obstacle.points) {
        val start = (obstacle.xs(i), obstacle.ys(i))
        val end = (obstacle.xs((i + 1) % obstacle.points), obstacle.ys((i + 1) % obstacle.points))
        if (isVertical(start, end)) { // can be left or right
          var segment = Segment(start._2, end._2)
          if (isLeft(continuousEnvCell, start)) {
            cardinalNeighbourhood(Left) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Left), segment)
          } else if (isRight(continuousEnvCell, start)) {
            segment = Segment(segment.b, segment.a) // need to mirror in case it's right boundary
            cardinalNeighbourhood(Right) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Right), segment)
          }
        } else if (isHorizontal(start, end)) { // can be top or bottom
          var segment = Segment(start._1, end._1)
          if (isTop(continuousEnvCell, start)) {
            cardinalNeighbourhood(Top) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Top), segment)
          } else if (isBottom(continuousEnvCell, start)) {
            segment = Segment(segment.b, segment.a) // need to mirror in case it's bottom boundary
            cardinalNeighbourhood(Bottom) = updateBoundaryWithObstacleSegment(cardinalNeighbourhood(Bottom), segment)
          }
        }
      }
    }

    for (obstacle <- continuousEnvCell.obstacles) {
      for (i <- 0 until obstacle.points) {
        val point = (obstacle.xs(i), obstacle.ys(i))
        if (isTopLeft(continuousEnvCell, point)) {
          diagonalNeighbourhood(TopLeft) = null
        } else if (isTopRight(continuousEnvCell, point)) {
          diagonalNeighbourhood(TopRight) = null
        } else if (isBottomRight(continuousEnvCell, point)) {
          diagonalNeighbourhood(BottomRight) = null
        } else if (isBottomLeft(continuousEnvCell, point)) {
          diagonalNeighbourhood(BottomLeft) = null
        }
      }
    }

    val newCellNeighbourhood = Neighbourhood(Map.from(cardinalNeighbourhood), Map.from(diagonalNeighbourhood))

    newCellNeighbourhood
  }

  private def updateBoundaryWithObstacleSegment(boundary: Boundary, segment: Segment): Boundary = {
    val boundaries: MutableMap[Segment, GridMultiCellId] = MutableMap.empty

    for ((existingSegment, neighbourId) <- boundary.boundaries) {
      if (existingSegment.a < segment.a && existingSegment.b > segment.b) {
        boundaries(Segment(existingSegment.a, segment.a)) = neighbourId
        boundaries(Segment(segment.b, existingSegment.b)) = neighbourId
      } else if (existingSegment.a < segment.a && existingSegment.b <= segment.b) {
        boundaries(Segment(existingSegment.a, segment.a)) = neighbourId
      } else if (existingSegment.a >= segment.a && existingSegment.b > segment.b) {
        boundaries(Segment(segment.b, existingSegment.b)) = neighbourId
      } else if (existingSegment.a >= segment.a && existingSegment.b <= segment.b) {
        // nothing
      }
    }

    Boundary(Map.from(boundaries))
  }
}
