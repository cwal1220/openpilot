import threading
import time
from typing import Any, Dict

from selfdrive.webui.settings import current_language, new_params, read_all, run_action as execute_action, schema, validate_registry, write_value

_DEBUG_LOCK = threading.Lock()
_DEBUG_SM = None


def _log(level: str, message: str) -> None:
  try:
    from selfdrive.swaglog import cloudlog
    getattr(cloudlog, level)(message)
  except Exception:
    print(f"webuid {level}: {message}", flush=True)


def _field(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def _lateral_state_dict(state: Any) -> Dict[str, Any]:
  if state is None:
    return {}
  result = {}
  for name in (
    "active", "p", "i", "d", "f", "output", "saturated", "error",
    "actualLateralAccel", "desiredLateralAccel", "steeringAngleDeg",
    "steeringAngleDesiredDeg", "angleError", "lqrOutput", "rateSetPoint",
    "accelSetPoint", "accelError", "selected",
  ):
    value = _field(state, name)
    if value is not None:
      result[name] = value
  return result


def debug_snapshot() -> Dict[str, Any]:
  global _DEBUG_SM
  try:
    import cereal.messaging as messaging
  except Exception as exc:
    return {"ok": False, "error": f"messaging unavailable: {exc}"}

  with _DEBUG_LOCK:
    if _DEBUG_SM is None:
      _DEBUG_SM = messaging.SubMaster(["controlsState"])
    _DEBUG_SM.update(0)
    cs = _DEBUG_SM["controlsState"]
    lateral = _field(cs, "lateralControlState")
    which = lateral.which() if lateral is not None else None
    state = getattr(lateral, which) if which else None
    return {
      "ok": True,
      "updated": bool(_DEBUG_SM.updated.get("controlsState", False)),
      "alive": bool(_DEBUG_SM.alive.get("controlsState", False)),
      "valid": bool(_DEBUG_SM.valid.get("controlsState", False)),
      "logMonoTime": int(_DEBUG_SM.logMonoTime.get("controlsState", 0)),
      "alerts": [
        _field(cs, "alertTextMsg1", ""),
        _field(cs, "alertTextMsg2", ""),
        _field(cs, "alertTextMsg3", ""),
      ],
      "controls": {
        "enabled": _field(cs, "enabled", False),
        "active": _field(cs, "active", False),
        "state": str(_field(cs, "state", "")),
        "lateralControlMethod": _field(cs, "lateralControlMethod", 0),
        "steerRatio": _field(cs, "steerRatio", 0.0),
        "steer": _field(cs, "steer", 0.0),
        "accel": _field(cs, "accel", 0.0),
        "steeringAngleDesiredDeg": _field(cs, "steeringAngleDesiredDeg", 0.0),
        "dynamicTRMode": _field(cs, "dynamicTRMode", 0),
        "dynamicTRValue": _field(cs, "dynamicTRValue", 0.0),
        "safetySpeed": _field(cs, "safetySpeed", 0.0),
        "gapBySpeedOn": _field(cs, "gapBySpeedOn", False),
      },
      "lateralState": {
        "which": which or "",
        "values": _lateral_state_dict(state),
      },
    }


def create_app():
  from flask import Flask, jsonify, render_template, request

  app = Flask(__name__)

  @app.get("/")
  def index():
    return render_template("index.html")

  @app.get("/api/schema")
  def get_schema():
    params = new_params()
    return jsonify(schema(current_language(params), params))

  @app.get("/api/params")
  def get_params():
    return jsonify({"values": read_all()})

  @app.post("/api/params/<key>")
  def set_param(key: str):
    body: Dict[str, Any] = request.get_json(silent=True) or {}
    if "value" not in body:
      return jsonify({"error": "missing value"}), 400
    try:
      value = write_value(key, body["value"])
    except KeyError:
      return jsonify({"error": "unknown key"}), 404
    except PermissionError:
      return jsonify({"error": "readonly key"}), 403
    except ValueError as exc:
      return jsonify({"error": str(exc)}), 400
    return jsonify({"key": key, "value": value})

  @app.post("/api/actions/<name>")
  def action_route(name: str):
    body: Dict[str, Any] = request.get_json(silent=True) or {}
    try:
      result = execute_action(name, body)
    except KeyError:
      return jsonify({"error": "unknown action"}), 404
    except ValueError as exc:
      return jsonify({"error": str(exc)}), 400
    return jsonify(result), (409 if result.get("status") == "busy" else 200)

  @app.get("/api/health")
  def health():
    return jsonify({"ok": True, "missing_registry_keys": validate_registry()})

  @app.get("/api/debug")
  def debug():
    return jsonify(debug_snapshot())

  return app


def main() -> None:
  try:
    from flask import Flask  # noqa: F401
  except Exception as exc:
    _log("error", f"disabled: Flask is not installed ({exc})")
    while True:
      time.sleep(60)

  missing = validate_registry()
  if missing:
    _log("warning", f"registry has keys missing from params.cc: {missing}")

  app = create_app()
  app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
  main()
