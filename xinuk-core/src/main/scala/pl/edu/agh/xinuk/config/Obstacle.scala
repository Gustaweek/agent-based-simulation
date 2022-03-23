package pl.edu.agh.xinuk.config

case class Obstacle(xs: Array[Int], ys: Array[Int], points: Int)

case class ObstacleSegment(a: (Int, Int), b: (Int, Int)) {
  def x: Double = b._1 - a._1

  def y: Double = b._2 - a._2
}
