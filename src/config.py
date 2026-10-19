"""Load a scenario YAML file into typed, validated dataclasses.

Every section of the schema in PLAN.md section 12 is parsed and validated, even
sections that later phases use. Unknown keys are errors (they are almost always
typos). Sections not used by the current phase are reported as info messages.
"""

from __future__ import annotations

import copy
import dataclasses
import types
import typing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Union, get_args, get_origin, get_type_hints

import yaml

# Sections the current build actually uses. Everything else is validated and ignored.
IMPLEMENTED_PHASE = 2
SECTIONS_USED = {
    "sim", "navigation", "drone_defaults", "drones", "scenario_events",
    "geofence", "confirmation", "association", "observation", "confidence", "track", "targets",
    "inspection",  # candidate_count, for the CONFIRM_HOLD point when the spot above the target is restricted
}  # fmt: skip


class ConfigError(ValueError):
    """Raised for any invalid configuration. The message names the offending key."""


Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


# --------------------------------------------------------------------------- schema


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.1
    duration: float = 400.0
    seed: int = 42
    headless: bool = False
    map_size: Point2 = (2000.0, 2000.0)

    def validate(self, p: str) -> None:
        _positive(self.dt, f"{p}.dt")
        _positive(self.duration, f"{p}.duration")
        _non_negative(self.seed, f"{p}.seed")
        _positive(self.map_size[0], f"{p}.map_size[0]")
        _positive(self.map_size[1], f"{p}.map_size[1]")


@dataclass(frozen=True)
class RestrictedZone:
    id: str
    polygon: list[Point2]

    def validate(self, p: str) -> None:
        _polygon(self.polygon, f"{p}.polygon")


@dataclass(frozen=True)
class GeofenceConfig:
    boundary: list[Point2] = field(
        default_factory=lambda: [(50.0, 50.0), (1950.0, 50.0), (1950.0, 1950.0), (50.0, 1950.0)]
    )
    margin: float = 15.0
    altitude_limits: Point2 = (15.0, 120.0)
    restricted_zones: list[RestrictedZone] = field(
        default_factory=lambda: [
            RestrictedZone("RZ-1", [(800.0, 600.0), (1100.0, 600.0), (1100.0, 1000.0), (800.0, 1000.0)])
        ]
    )

    def validate(self, p: str) -> None:
        _polygon(self.boundary, f"{p}.boundary")
        _non_negative(self.margin, f"{p}.margin")
        _range_pair(self.altitude_limits, f"{p}.altitude_limits", lo=0.0)
        _unique([z.id for z in self.restricted_zones], f"{p}.restricted_zones[].id")
        for i, z in enumerate(self.restricted_zones):
            z.validate(f"{p}.restricted_zones[{i}]")


@dataclass(frozen=True)
class Partition:
    start: float
    end: float
    groups: list[list[str]]

    def validate(self, p: str) -> None:
        _non_negative(self.start, f"{p}.start")
        if self.end <= self.start:
            raise ConfigError(f"{p}: end ({self.end}) must be after start ({self.start})")
        if len(self.groups) < 2:
            raise ConfigError(f"{p}.groups: need at least two groups")


@dataclass(frozen=True)
class NetworkConfig:
    comm_range: float = 1500.0
    latency: Point2 = (0.05, 0.2)
    packet_loss: float = 0.05
    max_message_age: float = 5.0
    partitions: list[Partition] = field(default_factory=list)

    def validate(self, p: str) -> None:
        _positive(self.comm_range, f"{p}.comm_range")
        _range_pair(self.latency, f"{p}.latency", lo=0.0)
        _probability(self.packet_loss, f"{p}.packet_loss")
        _positive(self.max_message_age, f"{p}.max_message_age")
        for i, part in enumerate(self.partitions):
            part.validate(f"{p}.partitions[{i}]")


@dataclass(frozen=True)
class HeartbeatConfig:
    period: float = 1.0
    miss_limit: int = 3

    def validate(self, p: str) -> None:
        _positive(self.period, f"{p}.period")
        _positive(self.miss_limit, f"{p}.miss_limit")


@dataclass(frozen=True)
class AllocationWeights:
    time: float = 1.0
    energy: float = 0.5
    battery: float = 0.3

    def validate(self, p: str) -> None:
        for name in ("time", "energy", "battery"):
            _non_negative(getattr(self, name), f"{p}.{name}")


@dataclass(frozen=True)
class AllocationConfig:
    weights: AllocationWeights = field(default_factory=AllocationWeights)
    round_period: float = 0.5
    consensus_rounds: int = 3
    allocation_timeout: float = 10.0
    max_reallocations: int = 2

    def validate(self, p: str) -> None:
        self.weights.validate(f"{p}.weights")
        _positive(self.round_period, f"{p}.round_period")
        _positive(self.consensus_rounds, f"{p}.consensus_rounds")
        _positive(self.allocation_timeout, f"{p}.allocation_timeout")
        _non_negative(self.max_reallocations, f"{p}.max_reallocations")


@dataclass(frozen=True)
class ConfirmationConfig:
    window: float = 5.0
    timeout: float = 15.0
    min_observations: int = 3
    min_mean_confidence: float = 0.6
    min_distinct_observers: int = 1
    detector_holds: bool = True
    hold_max: float = 8.0

    def validate(self, p: str) -> None:
        _positive(self.window, f"{p}.window")
        _positive(self.timeout, f"{p}.timeout")
        _positive(self.min_observations, f"{p}.min_observations")
        _probability(self.min_mean_confidence, f"{p}.min_mean_confidence")
        _positive(self.min_distinct_observers, f"{p}.min_distinct_observers")
        _non_negative(self.hold_max, f"{p}.hold_max")


@dataclass(frozen=True)
class AssociationConfig:
    radius: float = 30.0
    ignore_after_complete: float = 60.0

    def validate(self, p: str) -> None:
        _positive(self.radius, f"{p}.radius")
        _non_negative(self.ignore_after_complete, f"{p}.ignore_after_complete")


@dataclass(frozen=True)
class ObservationConfig:
    max_observation_age: float = 3.0
    obs_share_hz: float = 1.0

    def validate(self, p: str) -> None:
        _positive(self.max_observation_age, f"{p}.max_observation_age")
        _positive(self.obs_share_hz, f"{p}.obs_share_hz")


@dataclass(frozen=True)
class ConfidenceConfig:
    base: float = 0.95
    falloff_exp: float = 1.5
    noise_std: float = 0.05
    spurious_conf_range: Point2 = (0.2, 0.6)
    low_threshold: float = 0.4
    medium_threshold: float = 0.7

    def validate(self, p: str) -> None:
        _probability(self.base, f"{p}.base")
        _positive(self.falloff_exp, f"{p}.falloff_exp")
        _non_negative(self.noise_std, f"{p}.noise_std")
        _range_pair(self.spurious_conf_range, f"{p}.spurious_conf_range", lo=0.0, hi=1.0)
        _probability(self.low_threshold, f"{p}.low_threshold")
        _probability(self.medium_threshold, f"{p}.medium_threshold")
        if self.low_threshold >= self.medium_threshold:
            raise ConfigError(f"{p}: low_threshold must be below medium_threshold")


@dataclass(frozen=True)
class TrackConfig:
    smoothing: float = 0.3
    stale_after: float = 1.5
    lost_after: float = 3.0

    def validate(self, p: str) -> None:
        _probability(self.smoothing, f"{p}.smoothing")
        _positive(self.stale_after, f"{p}.stale_after")
        if self.lost_after <= self.stale_after:
            raise ConfigError(f"{p}: lost_after must be greater than stale_after")


@dataclass(frozen=True)
class ReacquireConfig:
    timeout: float = 20.0
    leg_step: float = 40.0
    timeout_action: Literal["cancel", "reallocate"] = "cancel"

    def validate(self, p: str) -> None:
        _positive(self.timeout, f"{p}.timeout")
        _positive(self.leg_step, f"{p}.leg_step")


@dataclass(frozen=True)
class InspectionConfig:
    duration: float = 10.0
    min_confidence: float = 0.6
    timer_on_loss: Literal["pause", "reset"] = "pause"
    standoff_alt: float = 60.0
    energy_margin: float = 1.2
    candidate_count: int = 16

    def validate(self, p: str) -> None:
        _positive(self.duration, f"{p}.duration")
        _probability(self.min_confidence, f"{p}.min_confidence")
        _positive(self.standoff_alt, f"{p}.standoff_alt")
        if self.energy_margin < 1.0:
            raise ConfigError(f"{p}.energy_margin must be >= 1.0")
        _positive(self.candidate_count, f"{p}.candidate_count")


@dataclass(frozen=True)
class NavigationConfig:
    cruise_alt: float = 60.0
    arrival_tolerance: float = 5.0
    alt_tolerance: float = 2.0
    replan_limit: int = 3
    # Extra offset (m) beyond the geofence margin for planner graph nodes, so a node is never
    # exactly on a buffered edge (which the edge rule treats as outside). Added in Phase 2.
    planner_clearance: float = 1.0

    def validate(self, p: str) -> None:
        _positive(self.cruise_alt, f"{p}.cruise_alt")
        _positive(self.planner_clearance, f"{p}.planner_clearance")
        _positive(self.arrival_tolerance, f"{p}.arrival_tolerance")
        _positive(self.alt_tolerance, f"{p}.alt_tolerance")
        _non_negative(self.replan_limit, f"{p}.replan_limit")


@dataclass(frozen=True)
class SafetyConfig:
    min_separation: float = 25.0
    separation_buffer: float = 10.0
    separation_horizon: float = 3.0

    def validate(self, p: str) -> None:
        _positive(self.min_separation, f"{p}.min_separation")
        _non_negative(self.separation_buffer, f"{p}.separation_buffer")
        _positive(self.separation_horizon, f"{p}.separation_horizon")


@dataclass(frozen=True)
class SensorConfig:
    radius: float = 80.0
    half_fov_deg: float = 45.0
    alt_band: Point2 = (20.0, 120.0)
    rate_hz: float = 2.0
    p_detect: float = 0.9
    position_noise_std: float = 3.0
    false_obs_rate: float = 0.0

    def validate(self, p: str) -> None:
        _positive(self.radius, f"{p}.radius")
        if not 0.0 < self.half_fov_deg < 90.0:
            raise ConfigError(f"{p}.half_fov_deg must be in (0, 90), got {self.half_fov_deg}")
        _range_pair(self.alt_band, f"{p}.alt_band", lo=0.0)
        _positive(self.rate_hz, f"{p}.rate_hz")
        _probability(self.p_detect, f"{p}.p_detect")
        _non_negative(self.position_noise_std, f"{p}.position_noise_std")
        _non_negative(self.false_obs_rate, f"{p}.false_obs_rate")


@dataclass(frozen=True)
class DroneParams:
    """Per-drone physical parameters. `drone_defaults` has this shape; each drone may override any key."""

    max_speed: float = 15.0
    climb_rate: float = 3.0
    comm_range: float = 1500.0
    capacity_wh: float = 100.0
    reserve_pct: float = 0.20
    critical_pct: float = 0.08
    k_h: float = 0.004
    k_c: float = 0.02
    k_d: float = 0.002
    hover_power: float = 0.05
    sensor: SensorConfig = field(default_factory=SensorConfig)

    def validate(self, p: str) -> None:
        _positive(self.max_speed, f"{p}.max_speed")
        _positive(self.climb_rate, f"{p}.climb_rate")
        _positive(self.comm_range, f"{p}.comm_range")
        _positive(self.capacity_wh, f"{p}.capacity_wh")
        _probability(self.reserve_pct, f"{p}.reserve_pct")
        _probability(self.critical_pct, f"{p}.critical_pct")
        if self.critical_pct >= self.reserve_pct:
            raise ConfigError(f"{p}: critical_pct must be below reserve_pct")
        for name in ("k_h", "k_c", "k_d", "hover_power"):
            _non_negative(getattr(self, name), f"{p}.{name}")
        self.sensor.validate(f"{p}.sensor")


@dataclass(frozen=True)
class DroneConfig:
    id: str
    pos: Point2
    alt: float
    battery_pct: float
    home: Point3
    patrol: list[Point3]
    params: DroneParams = field(default_factory=DroneParams)

    def validate(self, p: str) -> None:
        if not self.id:
            raise ConfigError(f"{p}.id must be a non-empty string")
        _non_negative(self.alt, f"{p}.alt")
        _probability(self.battery_pct, f"{p}.battery_pct")
        _non_negative(self.home[2], f"{p}.home[2]")
        if not self.patrol:
            raise ConfigError(f"{p}.patrol must contain at least one waypoint")
        for i, wp in enumerate(self.patrol):
            _non_negative(wp[2], f"{p}.patrol[{i}][2]")
        self.params.validate(f"{p}")


@dataclass(frozen=True)
class TargetConfig:
    id: str
    pos: Point2
    alt: float = 0.0
    appear_at: float = 0.0
    hidden_intervals: list[Point2] = field(default_factory=list)

    def validate(self, p: str) -> None:
        _non_negative(self.alt, f"{p}.alt")
        _non_negative(self.appear_at, f"{p}.appear_at")
        for i, iv in enumerate(self.hidden_intervals):
            _range_pair(iv, f"{p}.hidden_intervals[{i}]", lo=0.0)


# Required keys per scenario event type (beyond `t` and `type`).
SCENARIO_EVENT_FIELDS: dict[str, dict[str, type]] = {
    "fail_drone": {"drone": str},
    "set_battery": {"drone": str, "battery_pct": float},
    "external_cue": {"pos": list},
    "place_target": {"pos": list},
    "hide_target": {},  # optional "target": ground-truth id; default all
    "show_target": {},
}


@dataclass(frozen=True)
class ScenarioEventConfig:
    t: float
    type: str
    data: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Config:
    sim: SimConfig
    geofence: GeofenceConfig
    network: NetworkConfig
    heartbeat: HeartbeatConfig
    allocation: AllocationConfig
    confirmation: ConfirmationConfig
    association: AssociationConfig
    observation: ObservationConfig
    confidence: ConfidenceConfig
    track: TrackConfig
    reacquire: ReacquireConfig
    inspection: InspectionConfig
    navigation: NavigationConfig
    safety: SafetyConfig
    drone_defaults: DroneParams
    drones: list[DroneConfig]
    targets: list[TargetConfig]
    scenario_events: list[ScenarioEventConfig]
    name: str = "unnamed"
    info: tuple[str, ...] = ()  # informational messages produced while loading

    @property
    def drone_ids(self) -> list[str]:
        return sorted(d.id for d in self.drones)


# --------------------------------------------------------------------------- loading

_SECTION_TYPES: dict[str, type] = {
    "sim": SimConfig,
    "geofence": GeofenceConfig,
    "network": NetworkConfig,
    "heartbeat": HeartbeatConfig,
    "allocation": AllocationConfig,
    "confirmation": ConfirmationConfig,
    "association": AssociationConfig,
    "observation": ObservationConfig,
    "confidence": ConfidenceConfig,
    "track": TrackConfig,
    "reacquire": ReacquireConfig,
    "inspection": InspectionConfig,
    "navigation": NavigationConfig,
    "safety": SafetyConfig,
    "drone_defaults": DroneParams,
}
_LIST_SECTIONS = {"drones", "targets", "scenario_events"}


def load_config(path: str | Path) -> Config:
    path = Path(path)
    if not path.is_file():
        raise ConfigError(f"config file not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        try:
            raw = yaml.safe_load(fh)
        except yaml.YAMLError as exc:
            raise ConfigError(f"{path}: invalid YAML: {exc}") from exc
    return config_from_dict(raw if raw is not None else {}, name=path.stem)


def config_from_dict(raw: dict[str, Any], name: str = "unnamed") -> Config:
    if not isinstance(raw, dict):
        raise ConfigError("top level of the config must be a mapping")
    known = set(_SECTION_TYPES) | _LIST_SECTIONS
    unknown = sorted(set(raw) - known)
    if unknown:
        raise ConfigError(f"unknown top-level section(s): {', '.join(unknown)}")

    sections: dict[str, Any] = {}
    for key, cls in _SECTION_TYPES.items():
        sections[key] = _build(cls, raw.get(key) or {}, key)
        sections[key].validate(key)

    defaults_raw = raw.get("drone_defaults") or {}
    if "drones" not in raw or not raw["drones"]:
        raise ConfigError("drones: at least one drone is required")
    drones = [_build_drone(d, defaults_raw, f"drones[{i}]") for i, d in enumerate(_as_list(raw["drones"], "drones"))]
    _unique([d.id for d in drones], "drones[].id")

    targets = [_build(TargetConfig, t, f"targets[{i}]") for i, t in enumerate(_as_list(raw.get("targets") or [], "targets"))]
    for i, t in enumerate(targets):
        t.validate(f"targets[{i}]")
    _unique([t.id for t in targets], "targets[].id")

    drone_ids = {d.id for d in drones}
    events = [
        _build_scenario_event(e, drone_ids, f"scenario_events[{i}]")
        for i, e in enumerate(_as_list(raw.get("scenario_events") or [], "scenario_events"))
    ]
    for i, part in enumerate(sections["network"].partitions):
        for g in part.groups:
            for d in g:
                if d not in drone_ids:
                    raise ConfigError(f"network.partitions[{i}].groups: unknown drone '{d}'")

    _cross_validate(sections, drones)

    info = tuple(
        f"section '{s}' is validated but not used until a later phase (current phase: {IMPLEMENTED_PHASE})"
        for s in sorted(known - SECTIONS_USED)
        if s in raw
    )
    return Config(drones=drones, targets=targets, scenario_events=events, name=name, info=info, **sections)


def _cross_validate(sections: dict[str, Any], drones: list[DroneConfig]) -> None:
    lo, hi = sections["geofence"].altitude_limits
    nav: NavigationConfig = sections["navigation"]
    if not lo <= nav.cruise_alt <= hi:
        raise ConfigError(f"navigation.cruise_alt ({nav.cruise_alt}) is outside geofence.altitude_limits [{lo}, {hi}]")
    insp: InspectionConfig = sections["inspection"]
    if not lo <= insp.standoff_alt <= hi:
        raise ConfigError(
            f"inspection.standoff_alt ({insp.standoff_alt}) is outside geofence.altitude_limits [{lo}, {hi}]"
        )
    for d in drones:
        s_lo, s_hi = d.params.sensor.alt_band
        if s_hi < lo or s_lo > hi:
            raise ConfigError(f"drone {d.id}: sensor.alt_band [{s_lo}, {s_hi}] does not overlap altitude_limits")


def _build_drone(raw: Any, defaults_raw: dict[str, Any], p: str) -> DroneConfig:
    if not isinstance(raw, dict):
        raise ConfigError(f"{p} must be a mapping")
    own_keys = {"id", "pos", "alt", "battery_pct", "home", "patrol"}
    param_keys = {f.name for f in dataclasses.fields(DroneParams)}
    unknown = sorted(set(raw) - own_keys - param_keys)
    if unknown:
        raise ConfigError(f"{p}: unknown key(s): {', '.join(unknown)}")
    merged = _deep_merge(defaults_raw, {k: v for k, v in raw.items() if k in param_keys})
    params = _build(DroneParams, merged, p)
    own = {k: v for k, v in raw.items() if k in own_keys}
    drone = _build(DroneConfig, {**own, "params": {}}, p)
    drone = dataclasses.replace(drone, params=params)
    drone.validate(p)
    return drone


def _build_scenario_event(raw: Any, drone_ids: set[str], p: str) -> ScenarioEventConfig:
    if not isinstance(raw, dict):
        raise ConfigError(f"{p} must be a mapping")
    if "t" not in raw or "type" not in raw:
        raise ConfigError(f"{p}: 't' and 'type' are required")
    t = _convert(raw["t"], float, f"{p}.t")
    _non_negative(t, f"{p}.t")
    etype = raw["type"]
    if etype not in SCENARIO_EVENT_FIELDS:
        raise ConfigError(f"{p}.type: unknown event type '{etype}' (known: {', '.join(SCENARIO_EVENT_FIELDS)})")
    spec = SCENARIO_EVENT_FIELDS[etype]
    data = {k: v for k, v in raw.items() if k not in ("t", "type")}
    optional = {"target"} if etype in ("hide_target", "show_target") else set()
    extra = sorted(set(data) - set(spec) - optional)
    if extra:
        raise ConfigError(f"{p}: unexpected key(s) for '{etype}': {', '.join(extra)}")
    for key, typ in spec.items():
        if key not in data:
            raise ConfigError(f"{p}: '{etype}' requires '{key}'")
        if typ is list:
            data[key] = list(_convert(data[key], Point2, f"{p}.{key}"))
        else:
            data[key] = _convert(data[key], typ, f"{p}.{key}")
    if "drone" in data and data["drone"] not in drone_ids:
        raise ConfigError(f"{p}.drone: unknown drone '{data['drone']}'")
    if "battery_pct" in data:
        _probability(data["battery_pct"], f"{p}.battery_pct")
    return ScenarioEventConfig(t=t, type=etype, data=data)


def _build(cls: type, raw: Any, p: str) -> Any:
    """Build dataclass `cls` from a mapping, converting and type-checking each field."""
    if not isinstance(raw, dict):
        raise ConfigError(f"{p} must be a mapping, got {type(raw).__name__}")
    hints = get_type_hints(cls)
    fields = {f.name: f for f in dataclasses.fields(cls)}
    unknown = sorted(set(raw) - set(fields))
    if unknown:
        raise ConfigError(f"{p}: unknown key(s): {', '.join(unknown)}")
    kwargs = {}
    for name, f in fields.items():
        if name in raw:
            kwargs[name] = _convert(raw[name], hints[name], f"{p}.{name}")
        elif f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING:
            raise ConfigError(f"{p}: missing required key '{name}'")
    return cls(**kwargs)


def _convert(value: Any, typ: Any, p: str) -> Any:
    origin = get_origin(typ)
    if typ is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{p}: expected a number, got {value!r}")
        return float(value)
    if typ is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{p}: expected an integer, got {value!r}")
        return value
    if typ is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{p}: expected true/false, got {value!r}")
        return value
    if typ is str:
        if not isinstance(value, str):
            raise ConfigError(f"{p}: expected a string, got {value!r}")
        return value
    if origin is Literal:
        allowed = get_args(typ)
        if value not in allowed:
            raise ConfigError(f"{p}: must be one of {', '.join(map(str, allowed))}, got {value!r}")
        return value
    if origin is tuple:
        args = get_args(typ)
        if not isinstance(value, (list, tuple)) or len(value) != len(args):
            raise ConfigError(f"{p}: expected a list of {len(args)} values, got {value!r}")
        return tuple(_convert(v, a, f"{p}[{i}]") for i, (v, a) in enumerate(zip(value, args)))
    if origin is list:
        (arg,) = get_args(typ)
        if not isinstance(value, (list, tuple)):
            raise ConfigError(f"{p}: expected a list, got {value!r}")
        return [_convert(v, arg, f"{p}[{i}]") for i, v in enumerate(value)]
    if origin is dict:
        if not isinstance(value, dict):
            raise ConfigError(f"{p}: expected a mapping, got {value!r}")
        return dict(value)
    if origin in (Union, types.UnionType):
        args = get_args(typ)
        if value is None and type(None) in args:
            return None
        non_none = [a for a in args if a is not type(None)]
        return _convert(value, non_none[0], p)
    if dataclasses.is_dataclass(typ):
        return _build(typ, value, p)
    if typ is typing.Any:
        return value
    raise ConfigError(f"{p}: unsupported schema type {typ!r}")  # pragma: no cover


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _as_list(value: Any, p: str) -> list[Any]:
    if not isinstance(value, list):
        raise ConfigError(f"{p} must be a list")
    return value


# --------------------------------------------------------------------------- checks


def _positive(v: float, p: str) -> None:
    if v <= 0:
        raise ConfigError(f"{p} must be > 0, got {v}")


def _non_negative(v: float, p: str) -> None:
    if v < 0:
        raise ConfigError(f"{p} must be >= 0, got {v}")


def _probability(v: float, p: str) -> None:
    if not 0.0 <= v <= 1.0:
        raise ConfigError(f"{p} must be in [0, 1], got {v}")


def _range_pair(pair: Point2, p: str, lo: float | None = None, hi: float | None = None) -> None:
    a, b = pair
    if a > b:
        raise ConfigError(f"{p}: first value ({a}) must not exceed second ({b})")
    if lo is not None and a < lo:
        raise ConfigError(f"{p}: values must be >= {lo}")
    if hi is not None and b > hi:
        raise ConfigError(f"{p}: values must be <= {hi}")


def _polygon(points: list[Point2], p: str) -> None:
    # Structural check only. Geometric validity (self-intersection, overlap) is checked
    # with shapely by safety/geofence.py from Phase 2.
    if len(points) < 3:
        raise ConfigError(f"{p}: a polygon needs at least 3 points, got {len(points)}")


def _unique(ids: list[str], p: str) -> None:
    seen: set[str] = set()
    for i in ids:
        if i in seen:
            raise ConfigError(f"{p}: duplicate id '{i}'")
        seen.add(i)
