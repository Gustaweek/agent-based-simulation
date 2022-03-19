package pl.edu.agh.continuous.env.common.geometry

object Algorithms {

  def distancePointLine(x0: Double, y0: Double, x1: Double, y1: Double, x2: Double, y2: Double): Double = {
    // Point: x0, y0
    // Line: x1, y1 --- x2, y2
    Math.abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2
      - x1))
  }

  def distancePointLineSegment(x0: Double, y0: Double, x1: Double, y1: Double, x2: Double, y2: Double): Double = {
    import Math.{max, min}
    // https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    // Return minimum distance between line segment vw and point p
    val p = Vec2(x0, y0)
    val v = Vec2(x1, y1)
    val w = Vec2(x2, y2)
    val l2 = Line(v, w).lengthSq // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return Line(p, v).length // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    val t = max(0, min(1, ((p - v) dot (w - v)) / l2))
    val projection = v + (w - v) * t // Projection falls on the segment
    Line(p, projection).length
  }

  def distanceLineSegmentLineSegment(line1: Line, line2: Line): Double = {
    line1.intersect(line2) match {
      case Some(LineIntersection(_, true, true)) => 0
      case _ => Seq(
        line1.segmentDistance(line2.start),
        line1.segmentDistance(line2.end),
        line2.segmentDistance(line1.start),
        line2.segmentDistance(line1.end),
      ).min
    }
  }

  def projectPointOnLine(point: Vec2, line: Line): Vec2 = {
    line.dx match {
      case 0 =>
        val x = -line.C / line.A
        val y = point.y
        Vec2(x, y)
      case _ =>
        val m = line.dy / line.dx
        val b = line.start.y - m * line.start.x
        val x = (m * point.y + point.x - m * b) / (m * m + 1)
        val y = (m * m * point.y + m * point.x + b) / (m * m + 1)
        Vec2(x, y)
    }
  }

  final case class LineIntersection(pos: Vec2, onLine1: Boolean, onLine2: Boolean)

  def intersect(line1: Line, line2: Line): Option[LineIntersection] = {
    // if the lines intersect, the result contains the x and y of the intersection
    // (treating the lines as infinite) and booleans for
    // whether line segment 1 or line segment 2 contain the point
    // from: http://jsfiddle.net/justin_c_rounds/Gd2S2
    // ported to scala

    val line1Dx = line1.end.x - line1.start.x
    val line1Dy = line1.end.y - line1.start.y
    val line2Dx = line2.end.x - line2.start.x
    val line2Dy = line2.end.y - line2.start.y

    val denominator = (line2Dy * line1Dx) - (line2Dx * line1Dy)

    if (denominator == 0) return None // lines are parallel

    val startDx = line1.start.x - line2.start.x
    val startDy = line1.start.y - line2.start.y

    val numerator1 = (line2Dx * startDy) - (line2Dy * startDx)
    val numerator2 = (line1Dx * startDy) - (line1Dy * startDx)
    val a = numerator1 / denominator
    val b = numerator2 / denominator

    // if we cast these lines infinitely in both directions, they intersect here:
    val resultX = line1.start.x + (a * line1Dx)
    val resultY = line1.start.y + (a * line1Dy)
    /*
    // it is worth noting that this should be the same as:
    x = line2StartX + (b * (line2EndX - line2StartX))
    y = line2StartX + (b * (line2EndY - line2StartY))
    */
    // if line1 is a segment and line2 is infinite, they intersect if:
    val resultOnLine1 = a > 0 && a < 1
    // if line2 is a segment and line1 is infinite, they intersect if:
    val resultOnLine2 = b > 0 && b < 1
    // if line1 and line2 are segments, they intersect if both of the above are true
    Some(LineIntersection(Vec2(resultX, resultY), resultOnLine1, resultOnLine2))
  }

  def intersectCircleLine(circle: Circle, line: Line): Array[Vec2] = {
    // https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899
    // The intersection points are ordered by the distance from line.start
    val d = line.vector
    val f = line.start - circle.center

    val a = d dot d
    val b = (f * 2) dot d
    val c = (f dot f) - circle.r * circle.r

    val discriminantSq = b * b - 4 * a * c
    if (discriminantSq < 0) Array.empty[Vec2]
    else {
      val discriminant = Math.sqrt(discriminantSq)
      if (discriminant == 0) {
        // tangent
        val t = (-b - discriminant) / (2 * a)
        Array(line(t))
      } else {
        // two intersection points
        val t1 = (-b - discriminant) / (2 * a)
        val t2 = (-b + discriminant) / (2 * a)
        Array(line.start + (d * t1), line.start + (d * t2))
      }
    }
  }

  def circleOuterTangentCW(c1: Circle, c2: Circle): Option[Line] = {
    if ((c1 includes c2) || (c2 includes c1)) return None

    // outer tangent with corrected atan sign: https://en.wikipedia.org/wiki/Tangent_lines_to_circles#Outer_tangent
    val x1 = c1.center.x
    val y1 = c1.center.y
    val r1 = c1.r
    val x2 = c2.center.x
    val y2 = c2.center.y
    val r2 = c2.r

    val gamma = -Math.atan2(y2 - y1, x2 - x1) // atan2 sets the correct sign, so that all tangents are on the right
    // side of point order
    val beta = Math.asin((r2 - r1) / Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))
    val alpha = gamma - beta

    val x3 = x1 + r1 * Math.cos(Math.PI * 0.5 - alpha)
    val y3 = y1 + r1 * Math.sin(Math.PI * 0.5 - alpha)
    val x4 = x2 + r2 * Math.cos(Math.PI * 0.5 - alpha)
    val y4 = y2 + r2 * Math.sin(Math.PI * 0.5 - alpha)

    Some(Line(Vec2(x3, y3), Vec2(x4, y4)))
  }
}