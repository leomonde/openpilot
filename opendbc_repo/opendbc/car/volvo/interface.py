from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.values import CAR
#from panda import Panda

#ButtonType = car.CarState.ButtonEvent.Type
#EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "volvo"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvo)]
    #ret.safetyConfigs[-1].safetyParam |= Panda.SAFETY_VOLVO

    # ret.dashcamOnly = True
    ret.radarUnavailable = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 0.8

    return ret
