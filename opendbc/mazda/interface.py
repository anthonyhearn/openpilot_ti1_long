#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.longitudinal import enter_radar_programming_session
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS

MAZDA_LONG_SAFETY_PARAM = 1


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.alphaLongitudinalAvailable = candidate == CAR.MAZDA_CX5_2022
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda,
                                           MAZDA_LONG_SAFETY_PARAM if ret.openpilotLongitudinalControl else None)]
    ret.radarUnavailable = ret.openpilotLongitudinalControl or Bus.radar not in DBC[candidate]

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)

    ret.enableBsm = 0x477 in fingerprint[0]

    ret.steerActuatorDelay = 0.1
    if candidate in (CAR.MAZDA_CX5_2022,):
      ret.steerActuatorDelay = 0.07
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022,):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    if ret.openpilotLongitudinalControl:
      ret.startingState = True
      ret.startAccel = 1.0
      ret.vEgoStarting = 0.1
      ret.vEgoStopping = 0.25
      ret.longitudinalActuatorDelay = 0.3
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.2, 1.0, 0.8]
      ret.longitudinalTuning.kiBP = [0., 5., 20.]
      ret.longitudinalTuning.kiV = [0.18, 0.12, 0.08]

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send):
    if CP.openpilotLongitudinalControl:
      enter_radar_programming_session(can_recv, can_send)
