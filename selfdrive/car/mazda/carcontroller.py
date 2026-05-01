from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda.longitudinal import LONG_COMMAND_STEP, MazdaLongitudinalProfile, NEAR_STOP_ENTRY_SPEED, RADAR_BUS, TESTER_PRESENT_STEP, \
                                           create_longitudinal_messages, create_radar_tester_present, \
                                           hold_brake_accel, hold_latched_accel, near_stop_brake_accel
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaFlags
from opendbc.common.filter_simple import FirstOrderFilter
from opendbc.common.params import Params

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

CRZ_CTRL_LATCH_FRAMES = int(round(2.0 / DT_CTRL))
CRZ_CTRL_PASSIVE_FRAMES = int(round(9.6 / DT_CTRL))
CRZ_CTRL_RESUME_REACTIVATE_FRAMES = int(round(0.08 / DT_CTRL))
CRZ_INFO_RESUME_PHASE_FRAMES = int(round(0.20 / DT_CTRL))
HOLD_REQUEST_FRAMES = int(round(6.0 / DT_CTRL))
RESUME_RELEASE_FRAMES = int(round(0.5 / DT_CTRL))
MANUAL_OVERRIDE_BLEND_FRAMES = int(round(0.30 / DT_CTRL))
MANUAL_OVERRIDE_CRUISE_ACCEL = 0.10
MANUAL_OVERRIDE_BRAKE_STRONG_ACCEL = -0.50
MANUAL_OVERRIDE_BRAKE_ACCEL = -0.10


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.ti_apply_torque_last = 0  # TI support
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.long_counter = 0
    self.standstill_hold_frames = 0
    self.stop_intent_latched = False
    self.resume_release_frames = 0
    self.resume_crz_latched_frames = 0
    self.resume_phase_frames = 0
    self.resume_ctrl_active_prev = False
    self.virtual_resume_sent_latched = False
    self.resume_button_prev = False
    self.manual_override_prev = False
    self.manual_override_blend_frames = 0
    self.manual_override_start_accel = 0.0
    self.longitudinal_accel_last = 0.0
    # TI and filtering support
    self.acc_filter = FirstOrderFilter(0.0, .1, DT_CTRL, initialized=False)
    self.filtered_acc_last = 0
    self.long_active_last = False
    self.params_memory = Params("/dev/shm/params")

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0
    ti_apply_torque = 0  # TI support

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params)
      # TI support
      if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
        if CS.ti_lkas_allowed:
          ti_new_torque = int(round(CC.actuators.torque * self.params.TI_STEER_MAX))
          ti_apply_torque = apply_ti_steer_torque_limits(ti_new_torque, self.ti_apply_torque_last,
                                                         CS.out.steeringTorque, self.params)
    
    self.apply_torque_last = apply_torque
    self.ti_apply_torque_last = ti_apply_torque  # TI support

    virtual_resume_sent = False
    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
    else:
      self.brake_counter = 0
      if CS.out.standstill and CC.cruiseControl.resume and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
        virtual_resume_sent = True

    if self.CP.openpilotLongitudinalControl:
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      starting = CC.actuators.longControlState == LongCtrlState.starting
      # Do not treat tiny positive low-speed PID noise as a real restart
      # request. Only the explicit upstream starting phase should release the
      # synthetic HOLD clamp automatically.
      restart_requested = starting
      # Physical wheel RES survives radar suppression on CRZ_BTNS. For virtual
      # RES, do not start the synthetic unlatch path until the first RES frame
      # has actually been transmitted on the bus.
      if not CC.cruiseControl.resume or not CS.out.standstill:
        self.virtual_resume_sent_latched = False
      elif virtual_resume_sent:
        self.virtual_resume_sent_latched = True
      physical_resume_requested = bool(CS.accel_button)
      virtual_resume_requested = CC.cruiseControl.resume and self.virtual_resume_sent_latched
      effective_resume_requested = False
      release_hold_requested = False
      release_brake = False
      if not CC.longActive:
        self.standstill_hold_frames = 0
        self.stop_intent_latched = False
        self.resume_release_frames = 0
        self.resume_crz_latched_frames = 0
        self.resume_phase_frames = 0
        self.resume_ctrl_active_prev = False
        self.virtual_resume_sent_latched = False
      else:
        if stopping:
          self.stop_intent_latched = True

        hold_latched_ready = CS.out.standstill and self.standstill_hold_frames > HOLD_REQUEST_FRAMES
        # A physical wheel RES should always be able to ask Mazda to leave
        # HOLD. For virtual RES, require that we have both actually sent a RES
        # frame and reached the latched-hold phase so a transient shouldStop
        # flicker cannot prematurely release the synthetic hold.
        physical_resume_unlatch_requested = CS.out.standstill and physical_resume_requested and (not stopping or hold_latched_ready)
        virtual_resume_unlatch_requested = CS.out.standstill and virtual_resume_requested and hold_latched_ready
        resume_unlatch_requested = physical_resume_unlatch_requested or virtual_resume_unlatch_requested
        effective_resume_requested = resume_unlatch_requested
        resume_rising_edge = effective_resume_requested and not self.resume_button_prev
        release_brake = self.resume_release_frames > 0
        base_release_hold_requested = CC.cruiseControl.override or CS.out.gasPressed or restart_requested or release_brake

        if CS.out.standstill and self.stop_intent_latched and not base_release_hold_requested:
          self.standstill_hold_frames += 1
        else:
          self.standstill_hold_frames = 0

        if CS.out.standstill and not base_release_hold_requested and resume_rising_edge and self.standstill_hold_frames >= CRZ_CTRL_PASSIVE_FRAMES:
          # Stock briefly re-enables ACC in the latched-hold profile when RES is
          # first pressed, then drops back into the active stop-go profile.
          self.resume_crz_latched_frames = CRZ_CTRL_RESUME_REACTIVATE_FRAMES
        elif self.resume_crz_latched_frames > 0:
          self.resume_crz_latched_frames -= 1

        if resume_unlatch_requested:
          self.resume_release_frames = RESUME_RELEASE_FRAMES
        elif self.resume_release_frames > 0:
          self.resume_release_frames -= 1

        release_brake = self.resume_release_frames > 0
        release_hold_requested = base_release_hold_requested or resume_unlatch_requested or release_brake

        if release_hold_requested or (not CS.out.standstill and not stopping and CS.out.vEgo > NEAR_STOP_ENTRY_SPEED):
          self.stop_intent_latched = False

      # Only enter Mazda's synthetic stop-go/HOLD path when upstream has
      # actually committed to a stop. Once that happens, keep the stop intent
      # latched through the standstill/HOLD phases until a real restart or
      # driver override releases it.
      # A virtual RES press should happen while Mazda still sees the passive
      # stop-go hold state. Only release that synthetic hold once the car is
      # actually starting to move or the driver overrides with gas.
      stop_go_request = CC.longActive and self.stop_intent_latched and not release_hold_requested
      standstill_hold_request = stop_go_request and CS.out.standstill
      hold_latched = standstill_hold_request and self.standstill_hold_frames > HOLD_REQUEST_FRAMES
      brake_release_requested = release_hold_requested or effective_resume_requested

      crz_hold_latched = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_LATCH_FRAMES and \
                         (not effective_resume_requested or self.resume_crz_latched_frames > 0)
      # Stock resumes from passive hold by re-enabling ACC while the RES press
      # is active, instead of staying indefinitely in the passive-hold substate.
      crz_hold_passive = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_PASSIVE_FRAMES and not effective_resume_requested
      release_brake = self.resume_release_frames > 0
      crz_ctrl_resume_active = release_brake and CS.out.vEgo < self.CP.vEgoStarting and not crz_hold_latched and not crz_hold_passive
      if crz_ctrl_resume_active:
        if not self.resume_ctrl_active_prev:
          self.resume_phase_frames = CRZ_INFO_RESUME_PHASE_FRAMES
        elif self.resume_phase_frames > 0:
          self.resume_phase_frames -= 1
      else:
        self.resume_phase_frames = 0
      crz_info_resume_unlatching = crz_ctrl_resume_active and self.resume_phase_frames > 0
      self.resume_ctrl_active_prev = crz_ctrl_resume_active
      # Keep CRZ_INFO stop bits cleared through the whole synthetic brake-release
      # window. Otherwise Mazda sees positive accel while we still advertise an
      # active stop, which shows up in the logs as a failed restart handoff.
      crz_info_hold_request = stop_go_request and not (brake_release_requested or release_brake)

      accel = 0.0
      if CC.longActive:
        accel = CC.actuators.accel
        if release_brake:
          accel = max(accel, 0.0)
        elif CS.out.standstill:
          accel = hold_latched_accel() if hold_latched else hold_brake_accel()
        elif self.stop_intent_latched and not release_hold_requested and (stopping or CS.out.vEgo < NEAR_STOP_ENTRY_SPEED):
          accel = min(accel, near_stop_brake_accel(CS.out.vEgo))

      # Stock Mazda ACC stays logically engaged when the driver adds throttle.
      # Keep CRZ_INFO/CRZ_CTRL in an ACC-active state through that handoff and
      # blend the last planner accel toward neutral instead of dropping from one
      # stale active frame straight to zero/inactive on the next cycle.
      manual_override = CS.out.cruiseState.enabled and CS.out.gasPressed and not CS.out.standstill and not stop_go_request
      manual_override_rising = manual_override and not self.manual_override_prev
      if manual_override_rising:
        seed_accel = accel if CC.longActive else self.longitudinal_accel_last
        self.manual_override_start_accel = seed_accel
        self.manual_override_blend_frames = MANUAL_OVERRIDE_BLEND_FRAMES
      elif not manual_override:
        self.manual_override_blend_frames = 0
        self.manual_override_start_accel = 0.0

      if manual_override:
        blend_ratio = (self.manual_override_blend_frames / MANUAL_OVERRIDE_BLEND_FRAMES) if MANUAL_OVERRIDE_BLEND_FRAMES > 0 else 0.0
        accel = self.manual_override_start_accel * blend_ratio
        stop_go_request = False
        standstill_hold_request = False
        hold_latched = False
        brake_release_requested = False
        crz_hold_latched = False
        crz_hold_passive = False
        crz_ctrl_resume_active = False
        crz_info_resume_unlatching = False
        crz_info_hold_request = False
        if self.manual_override_blend_frames > 0:
          self.manual_override_blend_frames -= 1

      if self.frame % TESTER_PRESENT_STEP == 0:
        can_sends.append(create_radar_tester_present(RADAR_BUS))

      if self.frame % LONG_COMMAND_STEP == 0:
        long_active = CC.longActive or manual_override
        lead_visible = CC.hudControl.leadVisible
        crz_ctrl_profile_override = None
        crz_ctrl_acc_active_2_override = None
        crz_ctrl_radar_has_lead_override = None
        if manual_override:
          # Stock Mazda keeps ACC logically engaged during throttle override, but
          # it shifts into softer CRZ_CTRL substates than the normal alpha-long
          # follow/cruise templates. Reuse the closest stock templates and force
          # ACC_ACTIVE_2 low so the handoff matches the stock logs more closely.
          if accel >= MANUAL_OVERRIDE_CRUISE_ACCEL:
            crz_ctrl_profile_override = MazdaLongitudinalProfile.ENGAGED_CRUISE
          elif accel <= MANUAL_OVERRIDE_BRAKE_STRONG_ACCEL:
            crz_ctrl_profile_override = MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED
          elif accel <= MANUAL_OVERRIDE_BRAKE_ACCEL:
            crz_ctrl_profile_override = MazdaLongitudinalProfile.STOP_GO_HOLD
          else:
            crz_ctrl_profile_override = MazdaLongitudinalProfile.ENGAGED_FOLLOW
          crz_ctrl_acc_active_2_override = False
          crz_ctrl_radar_has_lead_override = True
        can_sends.extend(create_longitudinal_messages(RADAR_BUS, accel, self.long_counter,
                                                      long_active, lead_visible, CS.out.standstill,
                                                      hold_request=crz_info_hold_request,
                                                      crz_ctrl_hold_request=stop_go_request,
                                                      hold_latched=hold_latched,
                                                      crz_hold_latched=crz_hold_latched,
                                                      crz_hold_passive=crz_hold_passive,
                                                      crz_resume_active=crz_ctrl_resume_active,
                                                      crz_info_resume_unlatching=crz_info_resume_unlatching,
                                                      crz_ctrl_profile_override=crz_ctrl_profile_override,
                                                      crz_ctrl_acc_active_2_override=crz_ctrl_acc_active_2_override,
                                                      crz_ctrl_radar_has_lead_override=crz_ctrl_radar_has_lead_override,
                                                      v_ego=CS.out.vEgo))
        self.long_counter = (self.long_counter + 1) % 16
        self.longitudinal_accel_last = accel if long_active else 0.0
      self.manual_override_prev = manual_override
      self.resume_button_prev = effective_resume_requested
    else:
      self.manual_override_prev = False
      self.manual_override_blend_frames = 0
      self.manual_override_start_accel = 0.0
      self.longitudinal_accel_last = 0.0
      self.resume_button_prev = False

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas,
                                                      ti_apply_torque if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else None))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends