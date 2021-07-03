package pl.edu.agh.rabbits.algorithm

import pl.edu.agh.xinuk.algorithm.{StateUpdate, UpdateTag}
import pl.edu.agh.xinuk.model.Empty

object RabbitsUpdateTag {

  case object Stay extends UpdateTag

  case object LeaveRabbit extends UpdateTag {
    def apply(): StateUpdate = super.apply(Empty)
  }

  case object LeaveWolf extends UpdateTag {
    def apply(): StateUpdate = super.apply(Empty)
  }

  case object Arrive extends UpdateTag

  case object Spawn extends UpdateTag

  case object DieRabbit extends UpdateTag {
    def apply(): StateUpdate = super.apply(Empty)
  }

  case object DieWolf extends UpdateTag {
    def apply(): StateUpdate = super.apply(Empty)
  }

}
