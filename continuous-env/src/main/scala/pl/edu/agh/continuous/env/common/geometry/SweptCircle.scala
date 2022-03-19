package pl.edu.agh.continuous.env.common.geometry

final case class SweptCircle(start: Vec2,
                             end: Vec2,
                             r: Double) {
  def d: Double = r * 2

  def length: Double = line.length + d

  def line: Line = Line(start, end)

  def intersects(other: SweptCircle): Boolean = {
    line.segmentDistance(other.line) < (r + other.r)
  }

  def intersects(other: Line): Boolean = {
    line.segmentDistance(other) < r
  }

  def intersects(other: Circle): Boolean = {
    line.segmentDistance(other.center) < (r + other.r)
  }
}