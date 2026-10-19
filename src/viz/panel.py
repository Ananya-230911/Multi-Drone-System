"""Side panel (PLAN.md section 11.1): drones, target (owner's view + agreement), bids,
and a feed that switches between events and the message log.

Section 11.2: the target panel shows the owner's local view (or the lowest-ID
agent's view when there is no owner) and how many live agents hold the same
(target_id, state, epoch, owner). Disagreement is highlighted.
"""

from __future__ import annotations

import pygame

from src.viz import legend as L
from src.viz.renderer import UiState
from src.viz.snapshot import Snapshot

FEED_SKIP = {"WAYPOINT_REACHED", "OBSERVATION", "TRACK_FRESHNESS", "ROUTE_PLANNED",
             "MSG_SENT", "MSG_DELIVERED", "MSG_DROPPED"}  # fmt: skip
MSG_FILTERS = ("ALL", "HEARTBEAT", "TARGET_DETECTED", "OBSERVATION", "TARGET_STATE", "MERGE", "BID", "CLAIM", "ACK",
               "RELEASE", "DROPPED")  # fmt: skip
CONTROLS = [
    "SPACE pause  <-/-> speed  R reset  1-4 select  F fail  L legend",
    "click: ground-truth target  Shift+click: external cue  H hide/show",
    "TAB events/messages  T message filter  M message lines  G truth",
]


def target_focus(snap: Snapshot):
    """(target view, holder id, agreeing, live) for the most relevant target, or None."""
    live = [d for d in snap.drones if d.mode != "FAILED"]
    by_id: dict[str, list] = {}
    for d in live:
        for tv in d.targets:
            if tv.state != "CANCELLED":
                by_id.setdefault(tv.id, []).append((d.id, tv))
    if not by_id:
        return None
    tid = max(by_id, key=lambda k: (max(1 for _ in by_id[k]), max(tv.created_at for _, tv in by_id[k])))
    holders = dict(by_id[tid])
    owner = next((tv.owner for tv in holders.values() if tv.owner), None)
    holder = owner if owner in holders else min(holders)
    tv = holders[holder]
    key = (tv.id, tv.state, tv.epoch, tv.owner)
    agreeing = sum(1 for t in holders.values() if (t.id, t.state, t.epoch, t.owner) == key)
    return tv, holder, agreeing, len(live)


class Panel:
    def __init__(self, rect: pygame.Rect):
        self.rect = rect

    def draw(self, surf: pygame.Surface, snap: Snapshot, ui: UiState, fonts: dict[str, pygame.font.Font]) -> None:
        pygame.draw.rect(surf, L.PANEL_BG, self.rect)
        x0, y = self.rect.x + 12, self.rect.y + 8
        y = self._drones(surf, snap, ui, fonts, x0, y)
        y = self._target(surf, snap, fonts, x0, y + 4)
        y = self._bids(surf, snap, fonts, x0, y + 4)
        bottom = self.rect.bottom - 18 * len(CONTROLS) - 10
        if ui.feed == "messages":
            self._messages(surf, snap, ui, fonts, x0, y + 4, bottom)
        else:
            self._events(surf, snap, fonts, x0, y + 4, bottom)
        y = bottom + 4
        for line in CONTROLS:
            surf.blit(fonts["small"].render(line, True, L.MUTED), (x0, y))
            y += 18

    # ------------------------------------------------------------------ sections

    def _drones(self, surf, snap, ui, fonts, x0, y):
        surf.blit(fonts["title"].render("DRONES", True, L.TEXT), (x0, y))
        y += 22
        for d in snap.drones:
            if d.id == ui.selected:
                pygame.draw.rect(surf, (52, 58, 72), (self.rect.x + 4, y - 2, self.rect.width - 8, 30))
                pygame.draw.rect(surf, L.SELECT, (self.rect.x + 4, y - 2, self.rect.width - 8, 30), 1)
            mode = "PARKED HOME" if d.docked else d.mode
            task = f"owns {d.current_task}" if d.current_task else ""
            yield_txt = f"yield->{d.yielding_to}" if d.yielding_to else ""
            line = f"{d.id:<3}{mode:<12}{d.battery_pct * 100:5.1f}%{d.alt:4.0f}m peers {len(d.alive_peers)} {task} {yield_txt}"
            surf.blit(fonts["mono_small"].render(line, True, L.mode_colour(d.mode)), (x0, y))
            self._battery_bar(surf, x0, y + 17, self.rect.width - 36, d.battery_pct, d.reserve_pct)
            y += 31
        return y

    def _target(self, surf, snap, fonts, x0, y):
        focus = target_focus(snap)
        if focus is None:
            surf.blit(fonts["title"].render("TARGET", True, L.TEXT), (x0, y))
            surf.blit(fonts["small"].render("no target records yet", True, L.MUTED), (x0 + 80, y + 4))
            return y + 24
        tv, holder, agreeing, live = focus
        col = L.target_colour(tv.state)
        surf.blit(fonts["title"].render(f"TARGET {tv.id}  [{tv.state}]", True, col), (x0, y))
        y += 22
        age = snap.t - tv.created_at
        lines = [
            (f"confirmation {tv.confirmation:<9} conf {tv.last_confidence:4.2f} {tv.category:<6} {tv.freshness}", L.TEXT),
            (f"owner {tv.owner or '-':<4} epoch {tv.epoch}   view of {holder}   age {age:4.0f}s"
             + (f"   aliases {','.join(tv.aliases)}" if tv.aliases else ""), L.TEXT),
            (f"agents agreeing: {agreeing}/{live}", (120, 230, 120) if agreeing == live else L.WARN),
        ]  # fmt: skip
        for text, c in lines:
            surf.blit(fonts["mono_small"].render(text, True, c), (x0, y))
            y += 16
        return y

    def _bids(self, surf, snap, fonts, x0, y):
        focus = target_focus(snap)
        holder = focus[1] if focus else None
        dv = snap.drone(holder) if holder else None
        av = next((a for a in (dv.auctions if dv else ()) if focus and a.target_id == focus[0].id), None)
        title = f"BIDS (epoch {av.epoch}, {av.status}, view of {holder})" if av else "BIDS"
        surf.blit(fonts["title"].render(title, True, L.TEXT), (x0, y))
        y += 22
        if av is None:
            surf.blit(fonts["small"].render("no auction yet", True, L.MUTED), (x0, y))
            return y + 18
        surf.blit(fonts["mono_small"].render(f"{'':3}{'t_go s':>8}{'E_need':>8}{'batt_af':>8}{'cost':>8}  result", True, L.MUTED), (x0, y))
        y += 16
        for b in av.bids:
            win = b.drone == av.winner
            if b.feasible:
                line = f"{b.drone:<3}{b.t_go:8.1f}{b.e_need:8.2f}{b.batt_after:8.1f}{b.cost:8.2f}  {'WINNER' if win else 'ok'}"
            else:
                line = f"{b.drone:<3}{'':32}  infeasible: {b.reason}"
            col = (120, 230, 120) if win else (L.TEXT if b.feasible else L.MUTED)
            surf.blit(fonts["mono_small"].render(line, True, col), (x0, y))
            y += 16
        return y

    def _events(self, surf, snap, fonts, x0, y, bottom):
        surf.blit(fonts["title"].render("EVENTS  (TAB: messages)", True, L.TEXT), (x0, y))
        y += 22
        for e in reversed([e for e in snap.recent_events if e["type"] not in FEED_SKIP]):
            if y > bottom - 15:
                break
            surf.blit(fonts["small"].render(_describe(e), True, _event_colour(e)), (x0, y))
            y += 15

    def _messages(self, surf, snap, ui, fonts, x0, y, bottom):
        flt = MSG_FILTERS[ui.msg_filter % len(MSG_FILTERS)]
        stats = snap.network_stats or {}
        surf.blit(fonts["title"].render(f"MESSAGES [{flt}]  (T: filter)", True, L.TEXT), (x0, y))
        surf.blit(fonts["small"].render(f"sent {stats.get('sent', 0)} delivered {stats.get('delivered', 0)} "
                                        f"dropped {stats.get('dropped', 0)}", True, L.MUTED), (x0 + 300, y + 4))  # fmt: skip
        y += 22
        for m in reversed(snap.messages):
            if flt == "DROPPED" and m.status == "DELIVERED":
                continue
            if flt not in ("ALL", "DROPPED") and m.type != flt:
                continue
            if y > bottom - 15:
                break
            status = "" if m.status == "DELIVERED" else f"  DROPPED ({m.status})"
            tgt = f" {m.target_id}" if m.target_id else ""
            col = L.MUTED if status else _msg_colour(m.type)
            surf.blit(fonts["mono_small"].render(f"{m.t:7.1f} {m.sender}->{m.recipient} {m.type}{tgt}{status}", True, col), (x0, y))
            y += 15

    @staticmethod
    def _battery_bar(surf, x: int, y: int, w: int, pct: float, reserve: float) -> None:
        pygame.draw.rect(surf, L.BAR_BG, (x, y, w, 6))
        col = L.WARN if pct < reserve else (90, 200, 120)
        pygame.draw.rect(surf, col, (x, y, int(w * max(0.0, min(pct, 1.0))), 6))
        rx = x + int(w * reserve)
        pygame.draw.line(surf, L.TEXT, (rx, y - 2), (rx, y + 8), 1)  # reserve marker


def _msg_colour(mtype: str) -> tuple[int, int, int]:
    return L.MSG_COLOURS.get(mtype, L.TEXT)


def _describe(e: dict) -> str:
    d = e.get("data", {})
    t = f"{e['t']:7.1f}"
    tid = e.get("target_id", "")
    if e["type"] == "DRONE_MODE":
        return f"{t} {e['actor']} {d.get('from')} -> {d.get('to')} ({d.get('reason')})"
    if e["type"] == "SCENARIO_EVENT":
        who = f" {d['drone']}" if "drone" in d else (f" {d['target']}" if "target" in d else "")
        ok = "" if d.get("applied") else " [ignored]"
        return f"{t} scenario {d.get('type')}{who}{ok}"
    if e["type"] == "SAFETY_RETURN":
        e_home = d.get("e_home_wh")
        return f"{t} {e['actor']} SAFETY_RETURN {d.get('reason')}" + (f" (E_home {e_home:.1f} Wh)" if e_home is not None else "")
    if e["type"] in ("SAFETY_HOLD", "SAFETY_REPLAN", "SAFETY_LAND"):
        return f"{t} {e['actor']} {e['type']} {d.get('reason', d.get('result', ''))}"
    if e["type"] == "HOME_REACHED":
        nxt = "resume patrol" if d.get("resume_patrol") else "parked"
        return f"{t} {e['actor']} home, batt {d.get('battery_pct', 0) * 100:.1f}% -> {nxt}"
    if e["type"] == "TARGET_TRANSITION":
        return f"{t} {e['actor']} {tid} {d.get('from')} -> {d.get('to')} (row {d.get('row')}, {d.get('reason')})"
    if e["type"] == "CONFIRMATION_RESULT":
        via = f" via {d['via']}" if "via" in d else ""
        return f"{t} {e['actor']} {tid} {d.get('result')} ({d.get('observations_in_window')} obs, mean {d.get('mean_confidence', 0):.2f}){via}"
    if e["type"] == "TARGET_MERGED":
        return f"{t} {e['actor']} merged {d.get('alias')} into {d.get('survivor')}"
    if e["type"] == "TARGET_CREATED":
        src = f" (from {d['relayed_from']})" if "relayed_from" in d else ""
        return f"{t} {e['actor']} created {tid}{src}"
    if e["type"] == "BID_COMPUTED":
        if d.get("feasible"):
            return f"{t} {e['actor']} bid {d.get('bid'):.2f} for {tid}"
        return f"{t} {e['actor']} cannot bid for {tid}: {d.get('reason')}"
    if e["type"] == "ALLOCATION_COMMITTED":
        return f"{t} {e['actor']} committed owner {d.get('winner')} for {tid} ({d.get('how')})"
    if e["type"] == "ALLOCATION_FAILED":
        return f"{t} {e['actor']} allocation failed for {tid}: {d.get('reason')}"
    if e["type"] == "CLAIM_CONFLICT":
        return f"{t} {e['actor']} claim conflict on {tid}: {d.get('result', d.get('reason'))}"
    if e["type"] == "SEPARATION_YIELD":
        return f"{t} {e['actor']} yields to {d.get('to')} (separation)"
    if e["type"] == "ROUTE_REJECTED":
        return f"{t} {e['actor']} ROUTE_REJECTED {d.get('kind')} {d.get('reason')}"
    if e["type"] == "OBSERVATION_REJECTED":
        return f"{t} {e['actor']} observation rejected: {d.get('reason')}"
    return f"{t} {e['actor']} {e['type']} {tid}"


def _event_colour(e: dict) -> tuple[int, int, int]:
    if e["type"].startswith("SAFETY") or e["type"] in ("DRONE_FAILED", "ROUTE_REJECTED", "ALLOCATION_FAILED", "CLAIM_CONFLICT"):
        return L.WARN
    if e["type"] == "DRONE_MODE":
        return L.mode_colour(e["data"].get("to", ""))
    if e["type"] == "TARGET_TRANSITION":
        return L.target_colour(e["data"].get("to", ""))
    if e["type"] in ("ALLOCATION_COMMITTED", "BID_COMPUTED"):
        return (120, 230, 120)
    return L.TEXT
