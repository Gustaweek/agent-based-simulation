package pl.edu.agh.continuous.env.common

object EquationSolvers {

  def solveQuadraticEquation(a: Double, b: Double, c: Double): Either[String, QuadraticEquationResult] = {
    a match {
      case 0.0 => solveLinearEquation(b, c).map(result => QuadraticEquationSingleResult(result))
      case _ =>
        val delta = b * b - 4.0 * a * c
        delta match {
          case 0 => Right(QuadraticEquationSingleResult(-b / (2.0 * a)))
          case _ if delta < 0 => Left(s"a: $a; b: $b; c: $c; Δ: $delta")
          case _ => Right(QuadraticEquationDoubleResult(
            (-b - Math.sqrt(delta)) / (2.0 * a),
            (-b + Math.sqrt(delta)) / (2.0 * a)))
        }
    }
  }

  def solveLinearEquation(a: Double, b: Double): Either[String, Double] = {
    a match {
      case 0.0 => Left(s"a: $a; b: $b")
      case _ => Right(-b / a)
    }
  }

  sealed trait QuadraticEquationResult

  case class QuadraticEquationDoubleResult(result1: Double,
                                           result2: Double) extends QuadraticEquationResult with Seq[Double] {
    override def length: Int = 2

    override def iterator: Iterator[Double] = Iterator(result1, result2)

    override def apply(i: Int): Double = i match {
      case 0 => result1
      case 1 => result2
      case _ => throw new IndexOutOfBoundsException
    }
  }

  case class QuadraticEquationSingleResult(result: Double) extends QuadraticEquationResult
}
