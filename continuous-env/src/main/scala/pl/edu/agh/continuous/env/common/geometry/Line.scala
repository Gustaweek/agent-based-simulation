package pl.edu.agh.continuous.env.common.geometry

import pl.edu.agh.continuous.env.common.MathConstants

final case class Line(start: Vec2, end: Vec2) {
  def x1: Double = start.x

  def y1: Double = start.y

  def x2: Double = end.x

  def y2: Double = end.y

  def A: Double = if (isVertical) 1.0 else -dy / dx

  def B: Double = if (isVertical) 0.0 else 1.0

  def C: Double = -A * x1 - B * y1

  def reversed = new Line(end, start)

  def vector: Vec2 = end - start

  def normal: Vec2 = vector.normal

  def center: Vec2 = (start + end) * 0.5

  def leftOf(p: Vec2): Boolean = (vector cross (p - start)) > 0

  def rightOf(p: Vec2): Boolean = (vector cross (p - start)) <= 0

  def apply(t: Double): Vec2 = start + (vector * t)

  def distance(that: Vec2): Double = Algorithms.distancePointLine(that.x, that.y, x1, y1, x2, y2)

  def segmentDistance(that: Vec2): Double = Algorithms.distancePointLineSegment(that.x, that.y, x1, y1, x2, y2)

  def segmentDistance(that: Line): Double = Algorithms.distanceLineSegmentLineSegment(this, that)

  def segmentDistance(that: SweptCircle): Double = segmentDistance(that.line) - that.r

  def pointProjection(that: Vec2): Vec2 = Algorithms.projectPointOnLine(that, this)

  def intersect(that: Line): Option[Algorithms.LineIntersection] = Algorithms.intersect(this, that)

  def intersect(that: Circle): Array[Vec2] = Algorithms.intersectCircleLine(that, this)

  def segmentContains(that: Vec2): Boolean = segmentDistance(that) < MathConstants.epsilon

  def inflate(radius: Double): SweptCircle = SweptCircle(start, end, r = radius)

  def toSweptCircle: SweptCircle = inflate(radius = 0.0)

  def translate(translation: Vec2): Line = Line(start + translation, end + translation)

  def dx: Double = x2 - x1

  def dy: Double = y2 - y1

  def lengthSq: Double = dx * dx + dy * dy

  def length: Double = Math.sqrt(lengthSq)

  def isVertical: Boolean = dx == 0

  def isHorizontal: Boolean = dy == 0
}

object Line {
  def apply(x1: Double, y1: Double, x2: Double, y2: Double) = new Line(Vec2(x1, y1), Vec2(x2, y2))

  def apply(vec1: Vec2, vec2: Vec2) = new Line(vec1, vec2)
}