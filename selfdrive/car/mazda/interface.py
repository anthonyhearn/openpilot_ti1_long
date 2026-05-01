#!/usr/bin/env python3
from math import exp, fabs
import numpy as np

from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, LateralAccelFromTorqueCallbackType
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.longitudinal import enter_radar_programming_session
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, MazdaFlags, GEN1, GEN2, GEN3
from opendbc.common.params import Params

MAZDA_LONG_SAFETY_PARAM = 1

NON_LINEAR_TORQUE_PARAMS = {
  CAR.MAZDA_3_2019: (4.6, 0.6, 0.134, 0.3605),
  CAR.MAZDA_CX_30: (4.68689, 0.79999, 0.18244, 0.38763),
  CAR.MAZDA_CX_30_2023: (4.68689, 0.79999, 0.18244, 0.38763),
  CAR.MAZDA_CX_50: (4.68689, 0.79999, 0.18244, 0.38763)
}


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def get_lataccel_torque_siglin(self) -> float:
    """Calculate non-linear torque mapping using sigmoid + linear curves"""
    def torque_from_lateral_accel_siglin_func(lateral_acceleration: float) -> float:
      # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
      # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
      # This has big effect on the stability about 0 (noise when going straight)
      non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
      assert non_linear_torque_params, "The params are not defined"
      a, b, c, _ = non_linear_torque_params
      sig_input = a * lateral_acceleration
      sig = np.sign(sig_input) * (1 / (1 + exp(-fabs(sig_input))) - 0.5)
      steer_torque = (sig * b) + (lateral_acceleration * c)
      return float(steer_torque)

    lataccel_values = np.arange(-8.0, 8.0, 0.01)
    torque_values = [torque_from_lateral_accel_siglin_func(x) for x in lataccel_values]
    assert min(torque_values) < -1 and max(torque_values) > 1, "The torque values should cover the range [-1, 1]"
    return torque_values, lataccel_values

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    """Get the torque from lateral acceleration mapping function"""
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(lateral_acceleration, lataccel_values, torque_values)
      return torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
    """Get the lateral acceleration from torque mapping function"""
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def lateral_accel_from_torque_siglin(torque: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(torque, torque_values, lataccel_values)
      return lateral_accel_from_torque_siglin
    else:
      return self.lateral_accel_from_torque_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    """Get car parameters based on candidate and configuration"""
    ret.brand = "mazda"
    ret.carName = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda,
                                           MAZDA_LONG_SAFETY_PARAM if alpha_long else None)]
    ret.radarUnavailable = True
    ret.dashcamOnly = False
    
    # Set default longitudinal control
    ret.alphaLongitudinalAvailable = candidate == CAR.MAZDA_CX5_2022
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    ret.pcmCruise = True
    
    p = Params()
    
    # Manual transmission support
    if p.get_bool("ManualTransmission"):
      ret.flags |= MazdaFlags.MANUAL_TRANSMISSION.value
      ret.transmissionType = structs.CarParams.TransmissionType.manual
    else:
      ret.transmissionType = structs.CarParams.TransmissionType.automatic

    # GEN1 specific configuration
    if candidate in GEN1:
      ret.safetyConfigs[0].safetyParam |= 0x01  # FLAG_MAZDA_GEN1
      
      # Torque Interceptor support
      if p.get_bool("TorqueInterceptorEnabled"):
        print("Torque Interceptor Installed")
        ret.flags |= MazdaFlags.TORQUE_INTERCEPTOR.value
        ret.safetyConfigs[0].safetyParam |= 0x04  # FLAG_MAZDA_TORQUE_INTERCEPTOR
      
      # Radar Interceptor support
      if p.get_bool("RadarInterceptorEnabled"):
        ret.flags |= MazdaFlags.RADAR_INTERCEPTOR.value
        ret.experimentalLongitudinalAvailable = True
        ret.radarUnavailable = False
        ret.openpilotLongitudinalControl = True
        ret.startingState = True
        ret.longitudinalTuning.kpBP = [0., 5., 30.]
        ret.longitudinalTuning.kpV = [1.3, 1.0, 0.7]
        ret.longitudinalTuning.kiBP = [0., 5., 20., 30.]
        ret.longitudinalTuning.kiV = [0.36, 0.23, 0.17, 0.1]
        ret.safetyConfigs[0].safetyParam |= 0x08  # FLAG_MAZDA_RADAR_INTERCEPTOR
      
      # No Mazda Radar Cruise Control (missing CRZ_CTRL signal)
      if p.get_bool("NoMRCC"):
        ret.flags |= MazdaFlags.NO_MRCC.value
        ret.safetyConfigs[0].safetyParam |= 0x02  # FLAG_MAZDA_NO_MRCC
      
      # No Front Sensing Camera
      if p.get_bool("NoFSC"):
        ret.flags |= MazdaFlags.NO_FSC.value
        ret.safetyConfigs[0].safetyParam |= 0x10  # FLAG_MAZDA_NO_FSC

      ret.steerActuatorDelay = 0.1
      ret.enableBsm = True

    # GEN2 specific configuration
    elif candidate in GEN2:
      ret.safetyConfigs[0].safetyParam |= 0x20  # FLAG_MAZDA_GEN2
      ret.experimentalLongitudinalAvailable = True
      ret.openpilotLongitudinalControl = True
      ret.stopAccel = -.5
      ret.vEgoStarting = .2
      ret.longitudinalActuatorDelay = 0.35  # gas is 0.25s and brake looks like 0.5
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [0.0, 0.0, 0.0]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.1, 0.1]
      ret.startingState = True
      ret.steerActuatorDelay = 0.335

    # GEN3 specific configuration
    elif candidate in GEN3:
      ret.safetyConfigs[0].safetyParam |= 0x40  # FLAG_MAZDA_GEN3
      ret.experimentalLongitudinalAvailable = False
      ret.openpilotLongitudinalControl = False
      ret.steerActuatorDelay = 0.335
      if p.get_bool("ManualTransmission"):
        ret.flags |= MazdaFlags.MANUAL_TRANSMISSION.value

    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Set minimum steer speed for non-supported cars without TI
    if candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_3_2019, CAR.MAZDA_CX_30, CAR.MAZDA_CX_50, CAR.MAZDA_3_2023, CAR.MAZDA_CX_30_2023) and \
       not (ret.flags & MazdaFlags.TORQUE_INTERCEPTOR):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    """Get stock plus parameters for intelligent cruise button management"""
    ret.intelligentCruiseButtonManagementAvailable = True
    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send):
    """Initialize car interface"""
    if CP.openpilotLongitudinalControl:
      enter_radar_programming_session(can_recv, can_send)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    """Deinitialize car interface"""
    if CP.openpilotLongitudinalControl:
      # Mazda's radar faults if we explicitly request the default/active session
      # on teardown. Exiting cleanly is just stopping tester present and letting
      # the radar time out back to stock behavior on its own.
      return

  def _update(self, c, cp_sp):
    """Update car state and handle events"""
    ret, ret_sp = self.CS.update(self.can_parsers)
    
    # Create button events
    ret.buttonEvents = []

    # events
    events = self.create_common_events(ret)

    if self.CP.flags & MazdaFlags.GEN1:
      # Check for various GEN1-specific events
      if self.CS.lkas_disabled:
        events.add(EventName.lkasDisabled)
      elif self.CS.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

      # Torque Interceptor LKAS availability check
      if not self.CS.acc_active_last and not self.CS.ti_lkas_allowed:
        events.add(EventName.steerTempUnavailable)

    ret.events = events.to_msg()

    return ret, ret_sp