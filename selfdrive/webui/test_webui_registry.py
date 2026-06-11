#!/usr/bin/env python3
import re
import unittest
from pathlib import Path

from selfdrive.webui.settings import (
  K230_HEADLESS_EXCLUDED_PANELS,
  K230_HEADLESS_HIDDEN_KEYS,
  K230_HEADLESS_PANEL_OVERRIDES,
  K7_HEV_MODEL,
  SETTINGS,
  SETTINGS_BY_KEY,
  SETTING_DOCS,
  dynamic_options,
  run_action,
  schema,
  validate_registry,
  write_value,
)


class FakeParams:
  def __init__(self):
    self.values = {
      "LanguageSetting": b"main_ko",
      "DynamicTRSet": b"1.2,1.3",
      "DynamicTRSpd": b"10,20",
    }

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return self.values.get(key) == b"1"

  def put(self, key, value):
    self.values[key] = str(value).encode("utf8")

  def put_bool(self, key, value):
    self.values[key] = b"1" if value else b"0"

  def delete(self, key):
    self.values.pop(key, None)


class TestWebUiRegistry(unittest.TestCase):
  def test_writable_keys_are_known_params(self):
    self.assertEqual(validate_registry(), [])

  def test_no_duplicate_writable_keys(self):
    writable = [s.key for s in SETTINGS if s.type != "action" and not s.key.startswith("_")]
    self.assertEqual(len(writable), len(set(writable)))
    self.assertEqual(len(SETTINGS_BY_KEY), len(set(SETTINGS_BY_KEY)))

  def test_qt_settings_params_are_registered(self):
    files = [
      Path("selfdrive/ui/qt/offroad/settings.cc"),
      Path("selfdrive/ui/qt/widgets/opkr.cc"),
      Path("selfdrive/ui/qt/widgets/ssh_keys.cc"),
      Path("selfdrive/ui/qt/offroad/networking.cc"),
    ]
    text = "\n".join(path.read_text(errors="ignore") for path in files)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    keys = set()
    for pattern in [
      r'params\.(?:getBool|get|putBool|put|remove)\("([A-Za-z0-9_]+)"',
      r'Params\(\)\.(?:getBool|get|putBool|put|remove)\("([A-Za-z0-9_]+)"',
    ]:
      keys.update(re.findall(pattern, text))
    for block in re.findall(r"std::vector<std::tuple<QString, QString, QString, QString>> toggles\{(.*?)\n\s*\};", text, re.S):
      keys.update(re.findall(r'"([A-Za-z0-9_]+)"\s*,\s*tr\(', block))

    excluded = {
      "CalibrationParams", "CarParams", "DisableRadar_Allow", "DoReboot", "DoShutdown", "DoUninstall",
      "ExternalDeviceIP", "GitBranch", "GitCommit", "GitCommitRemote", "GitRemote", "GoogleMapEnabled",
      "HardwareSerial", "IMEI",
      "IsDriverViewEnabled", "IsOffroad", "LastAthenaPingTime", "LastUpdateTime", "LiveParameters",
      "MapboxEnabled", "NavSettingTime24h", "OPKRMapboxStyleSelect", "OPKRNaviSelect", "OSMOfflineUse",
      "OnRoadRefresh", "OpkrRunNaviOnBoot", "Passive", "TopTextView", "UpdateFailedCount",
    }
    excluded.update(K230_HEADLESS_HIDDEN_KEYS)
    missing = sorted(key for key in keys if key not in SETTINGS_BY_KEY and key not in excluded)
    self.assertEqual(missing, [])

  def test_k230_headless_schema_hides_inapplicable_qt_menus(self):
    data = schema("main_en", FakeParams())
    keys = {setting["key"] for setting in data["settings"]}
    actions = set(data["actions"])
    panels = {setting["panel"] for setting in data["settings"]}
    by_key = {setting["key"]: setting for setting in data["settings"]}
    self.assertTrue(K230_HEADLESS_EXCLUDED_PANELS.isdisjoint(data["panels"]))
    self.assertTrue(K230_HEADLESS_EXCLUDED_PANELS.isdisjoint(panels))
    self.assertTrue(K230_HEADLESS_HIDDEN_KEYS.isdisjoint(keys))
    self.assertNotIn("apply_k7_hev_defaults", keys)
    self.assertNotIn("apply_k7_hev_defaults", actions)
    self.assertTrue({"GsmRoaming", "GsmApn"}.isdisjoint(keys))
    for key, panel in K230_HEADLESS_PANEL_OVERRIDES.items():
      self.assertIn(key, by_key)
      self.assertEqual(by_key[key]["panel"], panel)

  def test_actions_are_k230_safe_subset(self):
    actions = set(schema()["actions"])
    self.assertEqual(actions, {
      "check_update",
      "fetch_github_ssh_keys",
      "git_pull",
      "reboot",
      "refresh",
      "remove_github_ssh_keys",
      "reset_calibration",
      "shutdown",
    })
    self.assertTrue({"DoUninstall", "delete_recordings", "panda_flash", "git_reset"}.isdisjoint(actions))

  def test_korean_schema_uses_qt_translations(self):
    data = schema("main_ko", FakeParams())
    self.assertEqual(data["messages"]["app_title"], "K230 설정")
    self.assertEqual(data["panel_titles"]["Device"], "장치")
    language = next(s for s in data["settings"] if s["key"] == "LanguageSetting")
    self.assertEqual(language["title"], "언어 변경")
    self.assertEqual([option["value"] for option in language["options"]], ["main_en", "main_ko"])
    hud = next(s for s in data["settings"] if s["key"] == "K230PreviewHudMode")
    self.assertEqual(hud["title"], "K230 프리뷰 HUD")
    self.assertEqual([option["label"] for option in hud["options"]], ["간결", "전체", "최소"])

  def test_unsupported_language_falls_back_to_english(self):
    data = schema("main_fr", FakeParams())
    self.assertEqual(data["language"], "main_en")
    self.assertEqual(data["messages"]["app_title"], "K230 Settings")
    language = next(s for s in data["settings"] if s["key"] == "LanguageSetting")
    self.assertEqual([option["value"] for option in language["options"]], ["main_en", "main_ko"])

  def test_car_model_options_have_hyundai_fallback(self):
    values = [option.value for option in dynamic_options("car_model", FakeParams())]
    self.assertIn(K7_HEV_MODEL, values)

  def test_schema_has_defaults_help_and_debug_panel(self):
    data = schema("main_en", FakeParams())
    self.assertIn("Debug", data["panels"])
    self.assertEqual(data["recommended_car_model"], K7_HEV_MODEL)
    car = next(s for s in data["settings"] if s["key"] == "CarModel")
    live = next(s for s in data["settings"] if s["key"] == "TorqueKp")
    self.assertEqual(car["default"], "")
    self.assertIn("car_helpers", car["details"])
    self.assertTrue(live["live"])
    self.assertIn("TorqueMaxLatAccel", live["value_guide"])
    self.assertNotIn("example", live)

  def test_help_is_localized_and_not_generic(self):
    english = schema("main_en", FakeParams())
    korean = schema("main_ko", FakeParams())
    bad_fragments = ["Boolean toggle stored", "Numeric Param", "Choose one of the listed options", "Current"]
    for data in (english, korean):
      self.assertIn("value_guide", data["messages"])
      self.assertNotIn("example", data["messages"])
      self.assertNotIn("current", data["messages"])
      for setting in data["settings"]:
        text = f"{setting.get('details', '')} {setting.get('value_guide', '')}"
        for fragment in bad_fragments:
          self.assertNotIn(fragment, text)

    camera_en = next(s for s in english["settings"] if s["key"] == "CameraOffsetAdj")
    camera_ko = next(s for s in korean["settings"] if s["key"] == "CameraOffsetAdj")
    self.assertIn("lane_planner", camera_en["details"])
    self.assertIn("lane_planner", camera_ko["details"])
    self.assertIn("millimeters", camera_en["value_guide"])
    self.assertIn("mm", camera_ko["value_guide"])
    self.assertEqual(korean["messages"]["details"], "기능")
    self.assertEqual(korean["messages"]["value_guide"], "값 / 단위")

  def test_driving_and_tuning_docs_describe_runtime_effects(self):
    effect_panels = {"Driving", "Tuning"}
    expected_keys = [setting.key for setting in SETTINGS if setting.panel in effect_panels]
    self.assertEqual([key for key in expected_keys if key not in SETTING_DOCS], [])

    english = schema("main_en", FakeParams())
    korean = schema("main_ko", FakeParams())
    bad_fragments = [
      "OPKR cruise/resume behavior setting",
      "OPKR speed-limit, safety-camera",
      "OPKR lane-change setting",
      "Hyundai/OPKR vehicle-integration compatibility",
      "Raw values follow the controller-side",
    ]
    for data in (english, korean):
      for setting in data["settings"]:
        if setting["panel"] not in effect_panels:
          continue
        text = f"{setting.get('details', '')} {setting.get('value_guide', '')}"
        for fragment in bad_fragments:
          self.assertNotIn(fragment, text)

    checks = {
      "OpkrAutoResume": ("RES_ACCEL", "재출발"),
      "OpkrLaneChangeSpeed": ("minimum speed", "최소 속도"),
      "SafetyCamDecelDistGain": ("earlier", "더 일찍"),
      "TorqueKp": ("stronger steering torque", "더 강한 조향 토크"),
      "OpkrVariableSteerMax": ("stronger steering torque", "더 강한 조향 토크"),
      "CruiseGap1": ("following time gap", "추종 시간 간격"),
      "CameraOffsetAdj": ("centers in the lane", "차선 안에서"),
    }
    for key, (english_fragment, korean_fragment) in checks.items():
      item_en = next(s for s in english["settings"] if s["key"] == key)
      item_ko = next(s for s in korean["settings"] if s["key"] == key)
      self.assertIn(english_fragment, f"{item_en['details']} {item_en['value_guide']}")
      self.assertIn(korean_fragment, f"{item_ko['details']} {item_ko['value_guide']}")

  def test_navigation_settings_are_excluded(self):
    keys = {setting.key for setting in SETTINGS}
    self.assertTrue({
      "ExternalDeviceIP",
      "GoogleMapEnabled",
      "MapboxEnabled",
      "OPKRMapboxStyleSelect",
      "OPKRNaviSelect",
      "OSMOfflineUse",
      "OpkrRunNaviOnBoot",
      "TopTextView",
    }.isdisjoint(keys))

  def test_csv_pair_validation(self):
    params = FakeParams()
    write_value("DynamicTRSpd", "30,40", params)
    with self.assertRaises(ValueError):
      write_value("DynamicTRSpd", "30,40,50", params)

  def test_action_dry_run(self):
    result = run_action("git_pull", {}, FakeParams(), dry_run=True)
    self.assertEqual(result["status"], "ok")
    self.assertEqual(result["action"], "git_pull")


if __name__ == "__main__":
  unittest.main()
