def create_steering_control(packer, bus, apply_steer, lkas_enabled, hca_mode):

  values = {
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": hca_mode if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  return packer.make_can_msg("HCA_1", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control, lat_active):
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Lampe_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Lampe_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Textbits": hud_alert,
  })

  return packer.make_can_msg("LDW_Status", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalt",      # ACC button, on/off
    "GRA_Typ_Hauptschalt",  # ACC button, momentary vs latching
    "GRA_Kodierinfo",       # ACC button, configuration
    "GRA_Sender",           # ACC button, CAN message originator
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Recall": resume,
  })

  return packer.make_can_msg("GRA_Neu", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if long_active:
    acc_control = 1
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    hud_status = 6
  elif long_active:
    hud_status = 3
  elif main_switch_on:
    hud_status = 2
  else:
    hud_status = 0

  return hud_status


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  values = {
    "ACS_Sta_ADR": acc_control,
    "ACS_StSt_Info": acc_enabled,
    "ACS_Typ_ACC": acc_type,
    "ACS_Anhaltewunsch": 0,
    "ACS_FreigSollB": acc_enabled,
    "ACS_Sollbeschl": accel if acc_enabled else 3.01,
    "ACS_zul_Regelabw": 0.2 if acc_enabled else 1.27,
    "ACS_max_AendGrad": 3.0 if acc_enabled else 5.08,
  }

  commands.append(packer.make_can_msg("ACC_System", bus, values))

  return commands

def create_epb_control(packer, bus, apply_brake, epb_enabled, stopping):

  values = {
    "EP1_Fehler_Sta": 0,
    "EP1_Sta_EPB": 0,
    "EP1_Spannkraft": 0,
    "EP1_Schalterinfo": 0,
    "EP1_Fkt_Lampe": 0,
    "EP1_Verzoegerung": apply_brake,                        #Brake request in m/s2
    "EP1_Freigabe_Ver": 1 if epb_enabled else 0,            #Allow braking pressure to build.
    "EP1_Bremslicht": 1 if apply_brake != 0 else 0,         #Enable brake lights
    "EP1_HydrHalten": 1 if stopping else 0,                 #Disengage DSG
    "EP1_AutoHold_aktiv": 1,                                #Disengage DSG
  }

  return packer.make_can_msg("EPB_1", bus, values)


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, personality_profile):
  values = {
    "ACA_StaACC": acc_hud_status,
    "ACA_Zeitluecke": 2,
    "ACA_V_Wunsch": set_speed,
    "ACA_gemZeitl": lead_distance,
    "ACA_PrioDisp": 3,
    # TODO: restore dynamic pop-to-foreground/highlight behavior with ACA_PrioDisp and ACA_AnzDisplay
    # TODO: ACA_kmh_mph handling probably needed to resolve rounding errors in displayed setpoint
  }

  return packer.make_can_msg("ACC_GRA_Anzeige", bus, values)
