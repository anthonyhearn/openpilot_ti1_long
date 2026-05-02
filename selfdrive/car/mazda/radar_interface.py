#!/usr/bin/env python3
import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.mazda.values import DBC, MazdaFlags

# 6 radar tracks on CAN IDs 0x361-0x366 (865-870), all at 10 Hz on bus 2.
# Each track reports distance, angle, and relative velocity.
# Empty slots are identified by sentinel values in all three fields.
# 0x361-0x364: stationary + moving objects — RELV_OBJ reliable
# 0x365-0x366: likely moving-vehicle-only slots — RELV_OBJ uses a different
#              encoding (not yet decoded). These are excluded because without
#              a valid vRel, radard's Kalman filter and lead-matching cannot
#              track them, and NaN velocity would poison downstream consumers.
RADAR_TRACK_ADDRS = list(range(0x361, 0x367))
RADAR_USABLE_ADDRS = set(range(0x361, 0x365))  # only tracks with reliable RELV
RADAR_TRIGGER_MSG = RADAR_TRACK_ADDRS[-1]  # 0x366 — last in the burst

# Sentinel values for empty slots (from DBC signal definitions)
SENTINEL_DIST = 4095 * 0.0625    # 255.9375 m — raw 4095
SENTINEL_ANG = 2046 * 0.015625   # 31.96875 deg — raw 2046
SENTINEL_RELV = -16 * 0.0625     # -1.0 m/s — raw -16


def _create_radar_can_parser(CP):
  """Create CAN parser for radar tracks"""
  if DBC[CP.carFingerprint]['radar'] is None:
    return None
  if not (CP.flags & MazdaFlags.RADAR_INTERCEPTOR):
    return None

  # Messages: RADAR_TRACK_361 through RADAR_TRACK_366 at 10 Hz
  messages = [(f"RADAR_TRACK_{addr:x}", 10) for addr in RADAR_TRACK_ADDRS]
  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 2)  # bus 2 = CAM bus


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.track_id = 0
    self.updated_messages = set()
    self.trigger_msg = RADAR_TRIGGER_MSG
    self.rcp = _create_radar_can_parser(CP)
    self.radar_off_can = CP.radarUnavailable or (self.rcp is None)

  def update(self, can_strings):
    if self.radar_off_can:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    # Only return data when we've received the last message in the burst
    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    
    # Check for CAN errors
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    # Parse each radar track
    for addr in RADAR_TRACK_ADDRS:
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      dist = msg['DIST_OBJ']
      ang = msg['ANG_OBJ']
      relv = msg['RELV_OBJ']

      # Empty slots have all three fields set to sentinel values.
      # Also skip tracks whose RELV encoding is not yet decoded — without
      # a valid vRel the downstream Kalman filter and MPC solver break.
      if dist == SENTINEL_DIST or ang == SENTINEL_ANG or relv == SENTINEL_RELV \
         or addr not in RADAR_USABLE_ADDRS:
        if addr in self.pts:
          del self.pts[addr]
        continue

      # Create or update track point
      if addr not in self.pts:
        self.pts[addr] = car.RadarData.RadarPoint.new_message()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      # Convert angle from degrees to radians and compute relative position
      azimuth = math.radians(ang)
      self.pts[addr].dRel = math.cos(azimuth) * dist      # longitudinal distance
      self.pts[addr].yRel = -math.sin(azimuth) * dist     # lateral distance (left is positive)
      self.pts[addr].vRel = relv                          # relative velocity
      self.pts[addr].aRel = float('nan')                  # acceleration not provided by radar
      self.pts[addr].yvRel = float('nan')                 # lateral velocity not provided
      self.pts[addr].measured = True

    ret.points = list(self.pts.values())
    return ret