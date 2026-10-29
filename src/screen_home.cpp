// screen_home.cpp — Home screen: CPU/GPU/RAM ring gauges.

#include <Arduino.h>
#include <lvgl.h>
#include "pin_config.h"
#include "pc_state.h"
#include "agent_status.h"
#include "screen_home.h"

lv_obj_t *cpuArc, *gpuArc, *ramArc;
lv_obj_t *cpuLabel, *gpuLabel, *ramLabel;
lv_obj_t *homeOfflineLabel;

// One ring = a dim full-circle "track" plus a colored value arc, swept
// clockwise starting from 12 o'clock — matches the watch-face mockup from
// the original UI brainstorm. LVGL's arc angle convention is 0°=3 o'clock
// like Arduino_GFX's was, so rotation=270 (i.e. -90) shifts the whole arc's
// coordinate frame so 0 visually lands at 12 o'clock; LV_ARC_MODE_NORMAL
// (the default) then grows the value indicator clockwise from there.
lv_obj_t *makeRingGauge(lv_obj_t *parent, int16_t diameter, int16_t thickness, lv_color_t color) {
  lv_obj_t *arc = lv_arc_create(parent);
  lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(arc, diameter, diameter);
  lv_obj_center(arc);
  lv_arc_set_bg_angles(arc, 0, 360);
  lv_arc_set_rotation(arc, 270);
  lv_arc_set_range(arc, 0, 100);
  // lv_arc_set_range() above is a no-op when the range already matches
  // lv_arc_create()'s own default (0-100, which it does) — it early-returns
  // before ever clamping the arc's internal value out of its "unset"
  // sentinel state. A freshly-created arc's indicator angles default to a
  // fixed 135°-270° sweep (LVGL's own generic-gauge look) and stay stuck
  // there — showing a visible partial arc — until something actually calls
  // lv_arc_set_value(). Every screen except Quick-jump happens to call that
  // via its own update*Screen() right after building, which is what masked
  // this; Quick-jump's ring never got a value at all and visibly showed the
  // leftover default arc (Brian: "partially filled light blue arc... what
  // is that for?"). Setting it explicitly here means every ring this helper
  // builds starts in a real "empty" state regardless of what its caller
  // does afterward.
  lv_arc_set_value(arc, 0);
  // True near-black, not a plain grey — the track should blend into the
  // true-black background per the original watch-face design (AMOLED
  // burn-in mitigation), just barely visible as a guide ring. Confirmed on
  // hardware with the equivalent color in the pre-LVGL version.
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x141414), LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, thickness, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(arc, false, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc, thickness, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc, false, LV_PART_INDICATOR);
  return arc;
}

lv_obj_t *makeRingLabel(lv_obj_t *parent, int16_t yOffset, lv_color_t color) {
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, yOffset);
  return label;
}

void updateHomeScreen() {
  // The arcs still take a 0-100 value regardless of unit — temperature and
  // percentage both happen to live in a sane 0-100 range for a ring gauge,
  // same as the mockup implicitly assumes (pct=42 drawn for "42°").
  lv_arc_set_value(cpuArc, (int)cpuTempC);
  lv_arc_set_value(gpuArc, (int)gpuTempC);
  lv_arc_set_value(ramArc, (int)ramPercent);
  lv_label_set_text_fmt(cpuLabel, "CPU %d°", (int)cpuTempC);
  lv_label_set_text_fmt(gpuLabel, "GPU %d°", (int)gpuTempC);
  lv_label_set_text_fmt(ramLabel, "RAM %d%%", (int)ramPercent);

  // PC-offline treatment (see pcConnected's own comment) — dim the rings
  // rather than hide them, so the layout doesn't jump, plus a label making
  // the "these numbers are stale/unavailable" state explicit. Gated on
  // PC_OFFLINE_DIM_ENABLED (see its own comment) — suppressed for now so
  // pcConnected's permanent false doesn't dim every screen during testing.
  bool showConnected = pcConnected || !PC_OFFLINE_DIM_ENABLED;
  lv_opa_t ringOpa = showConnected ? LV_OPA_COVER : LV_OPA_30;
  lv_obj_set_style_arc_opa(cpuArc, ringOpa, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(gpuArc, ringOpa, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(ramArc, ringOpa, LV_PART_INDICATOR);
  lv_obj_set_style_text_opa(cpuLabel, ringOpa, 0);
  lv_obj_set_style_text_opa(gpuLabel, ringOpa, 0);
  lv_obj_set_style_text_opa(ramLabel, ringOpa, 0);
  if (showConnected) {
    lv_obj_add_flag(homeOfflineLabel, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(homeOfflineLabel, LV_OBJ_FLAG_HIDDEN);
  }
}

void buildHomeScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE); // no scrollable content; keeps swipe-up gesture detection reliable

  // Diameters/thickness match the outer/inner radius pairs from the
  // pre-LVGL version (185/165, 160/140, 135/115) — same visual layout.
  cpuArc = makeRingGauge(scr, 370, 20, lv_color_hex(0xFF8C00));
  gpuArc = makeRingGauge(scr, 320, 20, lv_color_hex(0x0080FF));
  ramArc = makeRingGauge(scr, 270, 20, lv_color_hex(0x00C853));

  cpuLabel = makeRingLabel(scr, -44, lv_color_hex(0xFF8C00));
  gpuLabel = makeRingLabel(scr, 0, lv_color_hex(0x0080FF));
  ramLabel = makeRingLabel(scr, 44, lv_color_hex(0x00C853));

  homeOfflineLabel = lv_label_create(scr);
  lv_obj_set_style_text_font(homeOfflineLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(homeOfflineLabel, lv_color_hex(0x9AA5A3), 0);
  lv_label_set_text(homeOfflineLabel, "PC OFFLINE");
  lv_obj_align(homeOfflineLabel, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_add_flag(homeOfflineLabel, LV_OBJ_FLAG_HIDDEN); // shown by updateHomeScreen() once pcConnected is known

  // T60: agentStatusBadge sits at y=-90 — inside ramArc's inner radius
  // (115px), above the CPU/GPU/RAM label stack (which spans roughly y=-61
  // to 61 at font 28), clear of all three ring gauges and their ~5-10px
  // gaps. Not placed outside the rings (the gap between cpuArc's outer edge
  // and the panel bezel is only ~10px, too tight for a real badge) or on
  // idleScreen (T40-T51 fought hard to keep that screen free of any static
  // element for burn-in reasons — this stays Home-only, deliberately).
  agentStatusBadge = lv_obj_create(scr);
  lv_obj_remove_flag(agentStatusBadge, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(agentStatusBadge, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(agentStatusBadge, 34, 34);
  lv_obj_align(agentStatusBadge, LV_ALIGN_CENTER, 0, -90);
  lv_obj_set_style_radius(agentStatusBadge, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(agentStatusBadge, LV_OPA_TRANSP, 0); // ring only, no fill — same shape as speakerMuteRingPulse
  lv_obj_set_style_border_width(agentStatusBadge, 7, 0);
  lv_obj_set_style_pad_all(agentStatusBadge, 0, 0);
  lv_obj_add_flag(agentStatusBadge, LV_OBJ_FLAG_HIDDEN); // shown by updateAgentStatusBadge() once there's real data

  agentFlashOverlay = lv_obj_create(scr);
  lv_obj_remove_flag(agentFlashOverlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(agentFlashOverlay, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(agentFlashOverlay, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_center(agentFlashOverlay);
  lv_obj_set_style_radius(agentFlashOverlay, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(agentFlashOverlay, 0, 0);
  lv_obj_set_style_bg_opa(agentFlashOverlay, LV_OPA_TRANSP, 0); // fully transparent at rest; triggerAgentFlash() animates this
  lv_obj_set_style_pad_all(agentFlashOverlay, 0, 0);
  // Created last among Home's children — LVGL draws later-added siblings on
  // top, so this sits above the rings/labels/badge without needing an
  // explicit z-order call.

  updateHomeScreen();
  updateAgentStatusBadge();
}

// T44 (2026-08-26): the idle screen's clock's authoritative local-time
// source — see setupTimeSync()'s own comment (further down) for why this
// device's own clock is fixed to plain UTC and this offset (minutes east of
// UTC, e.g. -420 for Phoenix, AZ, no DST — pc-dash-app's own reasoning per
// the coordinator: correctly reducing Windows' DST rules to a POSIX TZ
// string is hard to verify and fails silently, twice a year, whereas an
// offset needs zero DST logic on either end and self-corrects every report)
// is the only local-time conversion applied, in updateIdleElement() below.
// Declared up here (rather than alongside the other sync-derived settings,
// dimDelayMs etc., much further down) because updateIdleElement() needs it
// and appears first in the file. Sourced from pcState via pollSyncState(),
// same "falls back to the last-synced/NVS value until a poll supplies a
// real one" pattern as volumeFuseMs/idleToHomeMs/dimDelayMs — defaults to 0
// (shows plain UTC) until the first real value lands. timeZoneName is
// cosmetic only (pc-dash-app's own wording) — never used for time math, not
// currently rendered anywhere; kept only so it's on hand if a future screen
// wants to display it.
int32_t utcOffsetMinutes = 0;
String timeZoneName = "";
