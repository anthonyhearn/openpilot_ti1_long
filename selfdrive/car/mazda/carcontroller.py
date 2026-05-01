from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda import LONG_COMMAND_STEP, MazdaLongitudinalProfile, NEAR_STOP_ENTRY_SPEED, RADAR_BUS, TESTER_PRESENT_STEP, \
                                            create_longitudinal_messages, create_radar_tester_present, \
                                           hold_brake_accel, hold_latched_accel, near_stop_brake_accel
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, MazdaFlags
from openpilot.common.realtime import ControlsTimer as Timer, DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params


VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# Longitudinal control constants
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
NEAR_STOP_ENTRY_SPEED = 0.5


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.ti_apply_steer_last = 0
    self.packer = CANPacker(dbc_name)
    self.brake_counter = 0
    self.frame = 0
    self.ccp = CarControllerParams(CP)
    self.hold_timer = Timer(6.0)
    self.hold_delay = Timer(.5) # delay before we start holding as to not hit the brakes too hard
    self.resume_timer = Timer(0.5)
    self.cancel_delay = Timer(0.07) # 70ms delay to try to avoid a race condition with stock system
    self.acc_filter = FirstOrderFilter(0.0, .1, DT_CTRL, initialized=False)
    self.filtered_acc_last = 0
    self.long_active_last = False
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    
    # Longitudinal control state
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
    self.long_counter = 0


  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    can_sends = []

    apply_steer = 0
    ti_apply_steer = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_steer = int(round(CC.actuators.steer * self.ccp.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorque, self.ccp)
      if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
        if CS.ti_lkas_allowed:
          ti_new_steer = int(round(CC.actuators.steer * self.ccp.TI_STEER_MAX))
          ti_apply_steer = apply_ti_steer_torque_limits(ti_new_steer, self.ti_apply_steer_last,
                                                    CS.out.steeringTorque, self.ccp)
    self.apply_steer_last = apply_steer
    self.ti_apply_steer_last = ti_apply_steer

    if self.CP.flags & MazdaFlags.GEN1:
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

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        if not self.CP.flags & MazdaFlags.NO_FSC:
          can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      if self.CP.flags & MazdaFlags.RADAR_INTERCEPTOR:
        hold = False
        if CS.out.standstill:
          hold = self.hold_timer.active()
        else:
          self.hold_timer.reset()

        if CC.longActive:
          raw_acc_output = CC.actuators.accel * 1150
          raw_acc_output = max(-1000, min(raw_acc_output, 1000))

          if self.params.get_bool("BlendedACC"):
            if self.params_memory.get_int("CEStatus"):
              self.acc_filter.update_alpha(abs(raw_acc_output-self.filtered_acc_last)/1000)
              filtered_acc_output = int(self.acc_filter.update(raw_acc_output))
            else:
              # we want to use the stock value in this case but we need a smooth transition.
              self.acc_filter.update_alpha(abs(CS.crz_info["ACCEL_CMD"]-self.filtered_acc_last)/1000)
              filtered_acc_output = int(self.acc_filter.update(CS.crz_info["ACCEL_CMD"]))

            CS.crz_info["ACCEL_CMD"] = int(filtered_acc_output)
            self.filtered_acc_last = filtered_acc_output
          else:
            acc_output = raw_acc_output

        if self.frame % 2 == 0:
          can_sends.extend(mazdacan.create_radar_command(self.packer, self.frame, CC.longActive, CS, hold))

    elif self.CP.flags & MazdaFlags.GEN2:
      virtual_resume_sent = False
      
      if CC.cruiseControl.cancel:
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
          virtual_resume_sent = True

      # Longitudinal control logic for GEN2
      if self.CP.openpilotLongitudinalControl:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        restart_requested = starting
        
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

        stop_go_request = CC.longActive and self.stop_intent_latched and not release_hold_requested
        standstill_hold_request = stop_go_request and CS.out.standstill
        hold_latched = standstill_hold_request and self.standstill_hold_frames > HOLD_REQUEST_FRAMES
        brake_release_requested = release_hold_requested or effective_resume_requested

        crz_hold_latched = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_LATCH_FRAMES and \
                           (not effective_resume_requested or self.resume_crz_latched_frames > 0)
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

        accel = 0.0
        if CC.longActive:
          accel = CC.actuators.accel
          if release_brake:
            accel = max(accel, 0.0)
          elif CS.out.standstill:
            accel = -0.1 if hold_latched else -0.1
          elif self.stop_intent_latched and not release_hold_requested and (stopping or CS.out.vEgo < NEAR_STOP_ENTRY_SPEED):
            accel = min(accel, -0.1)

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
          if self.manual_override_blend_frames > 0:
            self.manual_override_blend_frames -= 1

        self.manual_override_prev = manual_override
        self.resume_button_prev = effective_resume_requested
        self.longitudinal_accel_last = accel if CC.longActive else 0.0
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
        steer_required = steer_required and CS.lkas_allowed_speed
        can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.extend(mazdacan.create_steering_control(
      self.packer, self.CP,
      self.frame, apply_steer, CS.cam_lkas,
      ti_apply_steer if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else None
    ))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = apply_steer / self.ccp.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    Timer.tick()
    return new_actuators, can_sends
