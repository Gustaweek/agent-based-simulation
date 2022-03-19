package pl.edu.agh.continuous.env.common.geometry

import pl.edu.agh.continuous.env.common.MathUtils.DoubleExtensions

import scala.language.reflectiveCalls

final case class Vec2(x: Double, y: Double) {
  def unary_- : Vec2 = Vec2(-x, -y)

  def abs: Vec2 = Vec2(Math.abs(x), Math.abs(y))

  def +(that: Vec2): Vec2 = Vec2(this.x + that.x, this.y + that.y)

  def -(that: Vec2): Vec2 = Vec2(this.x - that.x, this.y - that.y)

  def *(a: Double): Vec2 = Vec2(this.x * a, this.y * a)

  def /(a: Double): Vec2 = Vec2(this.x / a, this.y / a)

  def dot(that: Vec2): Double = this.x * that.x + this.y * that.y

  def cross(that: Vec2): Double = this.x * that.y - this.y * that.x

  def lengthSq: Double = x * x + y * y

  def length: Double = Math.sqrt(lengthSq)

  def normalized: Vec2 = length match {
    case 0 => this
    case _ => this / length
  }

  def area: Double = x * y

  def normal: Vec2 = Vec2(y, -x)

  def angle: Double = Math.atan2(y, x)

  def clipLength(lowerBound: Double, upperBound: Double): Vec2 = {
    this.normalized * this.length.clip(lowerBound, upperBound)
  }

  def componentInDirectionOf(that: Vec2): Double = (this dot that) / that.length

  def projectionOn(that: Vec2): Vec2 = that * ((this dot that) / that.lengthSq)

  def toTuple: (Double, Double) = (x, y)

  def toArray: Array[Double] = {
    val a = new Array[Double](2)
    a(0) = x
    a(1) = y
    a
  }
}

object Vec2 {
  def apply(tuple: (Double, Double)) = new Vec2(tuple._1, tuple._2)

  def apply(x: Double) = new Vec2(x, x)

  def apply(v: {def x: Double; def y: Double}) = new Vec2(v.x, v.y)

  def apply(from: Vec2, to: Vec2): Vec2 = to - from

  def dim(v: {def width: Double; def height: Double}) = new Vec2(v.width, v.height)

  def zero = new Vec2(0, 0)

  def unitX = new Vec2(1, 0)

  def unitY = new Vec2(0, 1)

  def unit(angle: Double): Vec2 = Vec2(Math.cos(angle), Math.sin(angle))

  def dot(x1: Double, y1: Double, x2: Double, y2: Double): Double = x1 * x2 + y1 * y2

  def lengthSq(x: Double, y: Double): Double = x * x + y * y

  def length(x: Double, y: Double): Double = Math.sqrt(lengthSq(x, y))

  def normalize(length: Double, component: Double): Double = component / length
}
