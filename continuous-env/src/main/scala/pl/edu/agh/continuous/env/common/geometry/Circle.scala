package pl.edu.agh.continuous.env.common.geometry

final case class Circle(center: Vec2, r: Double) {
  def x: Double = center.x

  def y: Double = center.y

  def d: Double = r * 2

  def area: Double = Math.PI * r * r

  def sweep(vec: Vec2): SweptCircle = SweptCircle(center, center + vec, r)

  def centersDistance(that: Circle): Double = Line.apply(this.center, that.center).length

  def includes(that: Circle): Boolean = {
    val centersDistance = this.centersDistance(that)
    val rDiff = Math.abs(this.r - that.r)

    centersDistance < rDiff
  }

  def intersects(that: Circle): Boolean = {
    val rSum = this.r + that.r
    val centersDistance = this.centersDistance(that)

    centersDistance < rSum
  }

  def intersects(that: SweptCircle): Boolean = {
    that.intersects(this)
  }
}