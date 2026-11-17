from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR
from opendbc.car.tesla.radar_interface import RadarInterface

from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    # 2026+ Model Y (Juniper) runs a second gen HW4 stack: DAS_status is at 0x399 instead of 0x39b,
    # UI_warning (blinkers, buckle switch & doors) isn't sent at all, and DAS_settings is on the
    # party bus (it's only sent sporadically on the autopilot party bus, too rarely to parse)
    if 0x399 in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.HW4_GEN2.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.HW4_GEN2.value

      # VCFRONT_lighting (blinkers) is only on the VEHICLE bus
      if 0x3F5 in fingerprint[CANBUS.vehicle]:
        ret.flags |= TeslaFlags.HW4_GEN2_VEHICLE_BUS.value

    # Model X and HW 2.5 vehicles are missing DAS_settings
    elif 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar is never used on this branch: it's for a 2022 Model Y, which has no radar (only cars built up to
    # early 2021 have the Continental ARS4-B). Bus 1 can carry an unrelated RADAR_START_ADDR when the VEHICLE
    # bus is tapped into it, so don't detect the radar from the fingerprint either.
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    # Tesla expanded DAS_steeringControl->DAS_steeringControlType to 3 bits: first in the FSD 14 builds for HW4 around 10-26-2025,
    # then in the other HW4 builds around 03-02-2026, and for HW3 and HW2.5 with 2026.8.6 around 04-03-2026.
    # The values were pushed to the right by 1 bit and kept the same definitions, plus 4 = FSD was added.
    # These messages are only sent by the 3-bit firmware:
    # - HW3/HW4: DAS_redundantBrakingControl (0x489)
    # - HW2.5/HW3/HW4: DI_autonomyHealth (0x054)
    das_steering_3_bit = 0x489 in fingerprint[CANBUS.autopilot_party] or 0x054 in fingerprint[CANBUS.party]

    ret.dashcamOnly = (candidate in (CAR.TESLA_MODEL_X,) or  # dashcam only, pending find invalidLkasSetting signal
                       (not das_steering_3_bit and not docs))  # the car's software needs to be updated

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:

    if candidate == CAR.TESLA_MODEL_X:
      stock_cp.dashcamOnly = False

    if 0x3DF in fingerprint[1]:
      ret.flags |= TeslaFlagsSP.HAS_VEHICLE_BUS.value
      ret.safetyParam |= TeslaSafetyFlagsSP.HAS_VEHICLE_BUS

    return ret
