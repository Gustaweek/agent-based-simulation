package pl.edu.agh.rabbits.model

import pl.edu.agh.rabbits.config.RabbitsConfig
import pl.edu.agh.xinuk.config.XinukConfig
import pl.edu.agh.xinuk.model._

final case class Environment(rabbit: Option[Rabbit] = None, lettuce: Option[Lettuce] = None, wolf: Option[Wolf] = None) extends CellContents {
  override def generateSignal(iteration: Long)(implicit config: XinukConfig): Signal = {
    var signal: Signal = Signal(0d)
    if (rabbit.isDefined) signal = signal + config.asInstanceOf[RabbitsConfig].rabbitInitialSignal
    if (lettuce.isDefined) signal = signal + config.asInstanceOf[RabbitsConfig].lettuceInitialSignal
    if (wolf.isDefined) signal = signal + config.asInstanceOf[RabbitsConfig].wolfInitialSignal
    signal
  }
}