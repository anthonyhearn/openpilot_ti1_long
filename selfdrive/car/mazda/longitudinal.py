from __future__ import annotations

from enum import Enum
from typing import Optional
import numpy as np

from openpilot.common.numpy_fast import clip
from cereal import can

# Constants for Mazda longitudinal control
RADAR_ADDR = 0x764
RADAR_BUS = 0

CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C

CRZ_INFO_TEMPLATE = bytes.fromhex("01ffe20006800000")
CRZ_CTRL_TEMPLATE = bytes.fromhex("02010b0000000000")

LONG_COMMAND_STEP = 2
TESTER_PRESENT_STEP = 50

ACCEL_CMD_MAX = 2000.0
ACCEL_CMD_MIN = -2000.0
HOLD_BRAKE_CMD_TARGET = -1024.0
HOLD_LATCHED_CMD_TARGET = -1.0
NEAR_STOP_BRAKE_CMD_TARGET = -750.0
NEAR_STOP_ENTRY_SPEED = 1.0
ACTIVE_STOP_CHECKSUM_BIAS = 0x04

# Speed-dependent acceleration scaling
# Keep more authority at low/mid speed, soften at highway speeds
ACCEL_SCALE_UP_BP = [0.0, 4.2, 11.1, 22.2]
ACCEL_SCALE_UP_V = [1000.0, 1000.0, 950.0, 800.0]
ACCEL_SCALE_DOWN_BP = [0.0, 1.4, 5.6, 22.2]
ACCEL_SCALE_DOWN_V = [1200.0, 1000.0, 925.0, 950.0]


class MazdaLongitudinalProfile(str, Enum):
  """Mazda CRZ_CTRL profile templates"""
  STANDBY = "standby"
  ENGAGED_CRUISE = "engaged_cruise"
  ENGAGED_FOLLOW = "engaged_follow"
  STOP_GO_HOLD = "stop_go_hold"
  STOP_GO_HOLD_LATCHED = "stop_go_hold_latched"


CRZ_CTRL_TEMPLATES: dict[MazdaLongitudinalProfile, bytes] = {
  MazdaLongitudinalProfile.STANDBY: bytes.fromhex("02010b0000000000"),
  MazdaLongitudinalProfile.ENGAGED_CRUISE: bytes.fromhex("0a018b2000001000"),
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: bytes.fromhex("0a018b4000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD: bytes.fromhex("0a018b6000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED: bytes.fromhex("0a018b8000001000"),
}


def _interp(v_ego: float, bp: list[float], values: list[float]) -> float:
  """Linear interpolation helper"""
  return float(np.interp(v_ego, bp, values))


def accel_to_accel_cmd(accel: float, v_ego: float) -> int:
  """Convert acceleration (m/s^2) to raw CAN command"""
  if accel >= 0.0:
    scale = _interp(v_ego, ACCEL_SCALE_UP_BP, ACCEL_SCALE_UP_V)
  else:
    scale = _interp(v_ego, ACCEL_SCALE_DOWN_BP, ACCEL_SCALE_DOWN_V)
  
  return int(round(clip(accel * scale, ACCEL_CMD_MIN, ACCEL_CMD_MAX)))


def hold_brake_accel() -> float:
  """
  Brake acceleration for HOLD state.
  Stock HOLD keeps a strong negative CRZ_INFO command alive through the
  active stop/hold phase until the chassis hold latch takes over.
  """
  return HOLD_BRAKE_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def hold_latched_accel() -> float:
  """
  Brake acceleration for latched HOLD state.
  Once the chassis hold latch takes over, stock CRZ_INFO.ACCEL_CMD relaxes
  back near zero and the stop bits clear.
  """
  return HOLD_LATCHED_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def near_stop_brake_accel(v_ego: float) -> float:
  """
  Brake acceleration during near-stop phase.
  Stock stop-to-hold ramps into the final HOLD brake command before true
  standstill, rather than waiting until the speed bit drops to zero.
  """
  ratio = clip(v_ego / NEAR_STOP_ENTRY_SPEED, 0.0, 1.0)
  target = HOLD_BRAKE_CMD_TARGET + (NEAR_STOP_BRAKE_CMD_TARGET - HOLD_BRAKE_CMD_TARGET) * ratio
  return target / ACCEL_SCALE_DOWN_V[0]


def _compute_checksum(data: bytes, checksum_index: int = 7) -> int:
  """Compute inverted sum checksum for CAN messages"""
  return (0xFF - (sum(data[i] for i in range(len(data)) if i != checksum_index) & 0xFF)) & 0xFF


def _set_signal_value(data: bytearray, start_bit: int, length: int, 
                      value: int, is_signed: bool = False) -> None:
  """Set a signal value in CAN message data"""
  # Handle multi-byte signals
  for i in range(length):
    byte_idx = start_bit // 8 + i
    bit_in_byte = start_bit % 8
    
    if byte_idx < len(data):
      mask = (1 << length) - 1
      if is_signed and value < 0:
        value = (1 << length) + value
      
      data[byte_idx] = (data[byte_idx] & ~(mask << bit_in_byte)) | ((value >> (i * 8)) & mask) << bit_in_byte


def build_crz_info(accel: float, counter: int, long_active: bool, hold_request: bool, v_ego: float,
                   hold_latched: bool = False, acc_set_allowed: bool = True,
                   resume_unlatching: bool = False) -> bytes:
  """Build CRZ_INFO CAN message (0x21B)"""
  stopping_active = hold_request and not hold_latched
  
  raw = bytearray(CRZ_INFO_TEMPLATE)
  
  # ACCEL_CMD (bits 0-11)
  accel_cmd = accel_to_accel_cmd(accel, v_ego)
  raw[0] = (accel_cmd & 0xFF)
  raw[1] = ((raw[1] & 0xF0) | ((accel_cmd >> 8) & 0x0F))
  
  # ACC_ACTIVE (bit 12)
  if long_active:
    raw[1] |= 0x10
  else:
    raw[1] &= 0xEF
  
  # ACC_SET_ALLOWED (bit 13)
  if acc_set_allowed:
    raw[1] |= 0x20
  else:
    raw[1] &= 0xDF
  
  # STOPPING_MAYBE (bits 16-17)
  if stopping_active:
    raw[2] |= 0x03
  else:
    raw[2] &= 0xFC
  
  # RESUME_UNLATCHING_MAYBE (bit 18)
  if resume_unlatching:
    raw[2] |= 0x04
  else:
    raw[2] &= 0xFB
  
  # CTR1 (bits 20-23)
  raw[2] = (raw[2] & 0x0F) | ((counter % 16) << 4)
  
  # Checksum (byte 7)
  checksum_bias = ACTIVE_STOP_CHECKSUM_BIAS if stopping_active else 0
  raw[7] = (_compute_checksum(raw) + checksum_bias) & 0xFF
  
  return bytes(raw)


def select_profile(long_active: bool, lead_visible: bool, hold_request: bool,
                   crz_hold_latched: bool) -> MazdaLongitudinalProfile:
  """Select CRZ_CTRL profile based on current state"""
  if not long_active:
    return MazdaLongitudinalProfile.STANDBY
  if hold_request and crz_hold_latched:
    return MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED
  if hold_request:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if lead_visible:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  return MazdaLongitudinalProfile.ENGAGED_CRUISE


def build_crz_ctrl(long_active: bool, lead_visible: bool, hold_request: bool, hold_latched: bool,
                   crz_hold_latched: bool = False, crz_hold_passive: bool = False,
                   crz_resume_active: bool = False,
                   profile_override: Optional[MazdaLongitudinalProfile] = None,
                   acc_active_2_override: Optional[bool] = None,
                   radar_has_lead_override: Optional[bool] = None,
                   radar_lead_distance_override: Optional[int] = None,
                   acc_gas_override: Optional[int] = None) -> bytes:
  """Build CRZ_CTRL CAN message (0x21C)"""
  # Lead visibility includes hold/latch states
  lead_visible = lead_visible or hold_request or hold_latched or crz_hold_latched or crz_hold_passive
  
  # Select profile
  profile = profile_override if profile_override is not None else select_profile(long_active, lead_visible, hold_request, crz_hold_latched)
  raw = bytearray(CRZ_CTRL_TEMPLATES[profile])
  
  # CRZ_ACTIVE (bit 8)
  if long_active:
    raw[1] |= 0x01
  else:
    raw[1] &= 0xFE
  
  # ACC_ACTIVE_2 (bit 9)
  acc_active_2 = int(long_active and not crz_hold_passive) if acc_active_2_override is None else int(acc_active_2_override)
  if acc_active_2:
    raw[1] |= 0x02
  else:
    raw[1] &= 0xFD
  
  # RADAR_HAS_LEAD (bit 14)
  radar_has_lead = int(lead_visible) if radar_has_lead_override is None else int(radar_has_lead_override)
  if radar_has_lead:
    raw[1] |= 0x40
  else:
    raw[1] &= 0xBF
  
  # RADAR_LEAD_RELATIVE_DISTANCE (bits 16-19)
  if crz_hold_passive or crz_hold_latched:
    distance = 4
  elif hold_request or crz_resume_active:
    distance = 3
  else:
    distance = 2
  
  if radar_lead_distance_override is not None:
    distance = radar_lead_distance_override
  
  raw[2] = (raw[2] & 0xF0) | (distance & 0x0F)
  
  # ACC_GAS_MAYBE2 (bit 20)
  gas_maybe = 0
  if crz_hold_passive or crz_hold_latched:
    gas_maybe = 0
  elif hold_request or crz_resume_active:
    gas_maybe = 1
  
  if acc_gas_override is not None:
    gas_maybe = acc_gas_override
  
  if gas_maybe:
    raw[2] |= 0x10
  else:
    raw[2] &= 0xEF
  
  return bytes(raw)


def create_longitudinal_messages(bus: int, accel: float, counter: int, long_active: bool,
                                 lead_visible: bool, standstill: bool, *, 
                                 hold_request: bool = False,
                                 crz_ctrl_hold_request: Optional[bool] = None,
                                 hold_latched: bool = False, crz_hold_latched: bool = False,
                                 crz_hold_passive: bool = False,
                                 crz_resume_active: bool = False,
                                 crz_info_resume_unlatching: bool = False,
                                 crz_ctrl_profile_override: Optional[MazdaLongitudinalProfile] = None,
                                 crz_ctrl_acc_active_2_override: Optional[bool] = None,
                                 crz_ctrl_radar_has_lead_override: Optional[bool] = None,
                                 crz_ctrl_radar_distance_override: Optional[int] = None,
                                 crz_ctrl_acc_gas_override: Optional[int] = None,
                                 v_ego: float = 0.0) -> list[dict]:
  """Create longitudinal control CAN messages (CRZ_INFO and CRZ_CTRL)"""
  if crz_ctrl_hold_request is None:
    crz_ctrl_hold_request = hold_request

  crz_info_data = build_crz_info(accel, counter, long_active, hold_request, v_ego,
                                 hold_latched=hold_latched,
                                 resume_unlatching=crz_info_resume_unlatching)
  
  crz_ctrl_data = build_crz_ctrl(long_active, lead_visible, crz_ctrl_hold_request, hold_latched,
                                 crz_hold_latched=crz_hold_latched,
                                 crz_hold_passive=crz_hold_passive,
                                 crz_resume_active=crz_resume_active,
                                 profile_override=crz_ctrl_profile_override,
                                 acc_active_2_override=crz_ctrl_acc_active_2_override,
                                 radar_has_lead_override=crz_ctrl_radar_has_lead_override,
                                 radar_lead_distance_override=crz_ctrl_radar_distance_override,
                                 acc_gas_override=crz_ctrl_acc_gas_override)
  
  # Return as panda CAN format
  return [
    {"address": CRZ_INFO_ADDR, "busTime": 0, "dat": crz_info_data, "src": bus},
    {"address": CRZ_CTRL_ADDR, "busTime": 0, "dat": crz_ctrl_data, "src": bus},
  ]


def create_radar_tester_present(bus: int = RADAR_BUS) -> dict:
  """Create tester present message to keep radar awake"""
  # 0x62 = UDS Tester Present, 0x80 = suppress positive response
  return {"address": RADAR_ADDR, "busTime": 0, "dat": bytes([0x62, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), "src": bus}


# Radar session management (for future use)
def enter_radar_programming_session(can_recv, can_send) -> bool:
  """
  Enter radar programming session via UDS.
  This is for future radar parameter updates.
  """
  # TODO: Implement UDS diagnostic session if needed
  return True


def request_radar_default_session(can_recv, can_send) -> bool:
  """
  Request radar return to default session.
  """
  # TODO: Implement UDS diagnostic session if needed
  return True