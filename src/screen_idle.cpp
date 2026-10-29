// screen_idle.cpp — Burn-in-safe idle clock/date screen.

#include <Arduino.h>
#include <lvgl.h>
#include <WebSerial.h>
#include <time.h> // T40: struct tm/strftime for the idle screen's clock
#include "screen_home.h"
#include "screen_idle.h"

// Burn-in-mitigation idle screen (T40, 2026-08-25). Visual treatment redone
// for T41 (2026-08-25), T42 (2026-08-26), T43 (2026-08-26), T47 (2026-08-26),
// T49 (2026-08-26), T50 (2026-08-26), and T51 (2026-08-26) — each round
// Brian's own direct feedback after living with the previous one on the
// physical device. See ECOSYSTEM_PLAN.md's "Burn-in mitigation" section for
// the original design background.
//
// T51, Brian's own explicit answer to T50's residual-overlap tradeoff:
// don't just accept a compromised (still-possibly-overlapping) placement at
// whatever font size a cycle happened to randomly pick — first check
// whether a *smaller* font in the pool can find a fully clear spot instead,
// and only fall all the way back to a compromised placement (at the
// pool's smallest font) once every size has been tried and failed.
// pickIdleFontAndPosition() (below) now owns choosing the font *and* the
// position together — walks from the randomly-picked font size down
// through the pool, retrying pickIdlePosition() (which now reports whether
// it found a truly perfect candidate, not just a best-effort one) at each
// size until one succeeds or the smallest size itself becomes the final,
// most-forgiving attempt.
//
// T50: Brian kept seeing a plain overlap even after T49 — confirmed
// steady-state, not boot-adjacent (so not a T49 recurrence; that fix
// structurally holds every boot, verified by grepping for anything else
// that touches IdleElement state — nothing does). Re-derived the actual
// mechanism from this font's real bundled glyph-width table rather than
// guessing further: at 48pt, the date string is wide enough (~300px) that
// its circle-safe position range shrinks to roughly a 30px radius around
// the panel's center — small enough that Time's own (much larger) valid
// region can genuinely leave *zero* circle-safe, non-overlapping candidate
// for Date to find in `IDLE_POSITION_ATTEMPTS` tries. T43/T47's old 3-tier
// scheme handled that failure badly — its last tier accepted the first
// circle-safe random point with *no* regard for the other element,
// including landing exactly on top of it. Replaced with a best-effort
// search (pickIdlePosition() below) that scores every sampled candidate by
// actual clearance from the other element and keeps the best one, instead
// of an unconstrained one, when no perfect candidate exists — see that
// function's own comment for the full reasoning and idleBoxClearance() for
// the scoring itself.
//
// T49 fixed the *actual* remaining overlap bug — not a subtle mid-transition
// timing case (that was T47's fix, and a reasonable first hypothesis, but
// wrong for this report): `hasPosition` used to start `false` for *both*
// elements at boot, so whichever one happened to transition first (a coin
// flip — each element's very first dwell is independently randomized,
// nothing orders them) saw `other.hasPosition == false` and skipped the
// overlap check entirely, landing anywhere — including right on top of the
// other element's still-visible build-time placeholder ("--:--" for time).
// One element's-worth of bad luck per boot was enough, and this project
// reboots on every single flash, so it was hitting roughly every other
// boot. Fixed by seeding both elements with a real, accurate initial box
// (setIdleElementInitialBox(), called from buildIdleScreen()) instead of
// leaving hasPosition false until each element's first real transition.
//
// T47 fixed the real remaining gap in T43's overlap fix: T43's collision
// check only ever compared a fresh position against the *other* element's
// **current** box, but that element's just-vacated **previous** box is still
// visibly on screen too, cross-fading out over IDLE_FADE_MS
// (idleFadeAnimCb) — a transition landing during that window could still
// clip right onto the fading-out label undetected. Fixed by having each
// IdleElement remember that previous box (prevPosX/Y/halfW/halfH,
// fadeOutUntilMs — see its own comment) and checking both boxes, not just
// the current one, in pickIdlePosition(). Also added light WebSerial
// debug logging (only fires when a softer constraint actually gets relaxed)
// so a future overlap report can show whether real crowding, not this
// now-fixed gap, is the cause.
//
// T43 fixed three real (not cosmetic) bugs in T42's simultaneous-date+time
// design:
// 1. **Text sometimes clipped off-screen.** T42's IDLE_POSITION_POOL was a
//    fixed set of offsets hand-tuned by eyeballing one font/string combo,
//    never checked against the *actual* rendered box of whatever font/string
//    a given cycle happened to pick, and never checked against the panel's
//    true round shape — a position inside the square LVGL coordinate frame
//    can still clip against the physical circular bezel.
// 2. **Date and time landed exactly on top of each other** — the
//    no-collision-check gap T42's writeup flagged as a theoretical
//    possibility, confirmed to actually happen in real use.
// 3. **An element faded back in at the same spot it just left** —
//    independent per-cycle random selection doesn't exclude the
//    immediately-previous pick.
//
// Fix: replaced the fixed pool with real per-cycle rejection sampling
// (pickIdlePosition() below) against the box LVGL actually measures for
// whatever font/string this cycle picked (lv_text_get_size()) — checked
// against the panel's real circular boundary (idleBoxFitsCircle(), all 4
// box corners, not just the center point), against this element's own
// immediately-previous position, and against the other element's current
// position (idleBoxesTooClose()). See IdleElement/pickIdlePosition's own
// comments for exactly how.
//
// T42 carryovers, unchanged: date and time both shown at once, each its own
// independent element/timing (IDLE_DWELL_MIN_MS/MAX_MS), dropped 22/24pt
// fonts. T41 carryovers, unchanged: brightness (DISPLAY_BRIGHTNESS_IDLE), no
// separate pixel-shift mechanism (redundant once content/font/position all
// vary independently), cross-fade via idleFadeAnimCb (plain opacity anim on
// a label, same proven-safe technique as speakerMutePulseAnimCb/
// nowPlayingGlowAnimCb — not an lv_arc, so it doesn't run into HARDWARE.md's
// animated-arc hazard), and no live-ticking seconds display (a full-panel
// redraw every second indefinitely while idle is exactly what HARDWARE.md's
// "full-frame redraws only" rule warns against).
lv_obj_t *idleScreen;

// Each element (time, date) is its own independent two-label cross-fade
// pair — a single lv_label can't cross-dissolve between two different text
// strings on its own (no per-glyph interpolation), so each element keeps two
// overlapping labels and alternates which one is "incoming" (fading in at a
// freshly-picked font/position) vs. "outgoing" (fading out wherever it
// already was).
struct IdleElement {
  lv_obj_t *labelA;
  lv_obj_t *labelB;
  bool aIsVisible;
  uint32_t lastTransitionMs;
  uint32_t dwellMs; // this element's current wait — re-rolled every transition, see IDLE_DWELL_MIN_MS/MAX_MS
  // T43: the position + measured half-extents of whichever label is
  // actually visible right now. The *other* element reads these to avoid
  // landing on top of this one; this element reads its own previous values
  // (before overwriting them) to avoid re-landing on the same spot.
  // hasPosition is false until this element's first real transition has
  // actually placed something — nothing to avoid-repeat or avoid-collide
  // against before that.
  int16_t posX, posY;
  int16_t halfW, halfH;
  bool hasPosition;
  // T47: the box this element occupied *before* its most recent transition.
  // T43's collision check only ever compared against `other`'s *current*
  // (posX/posY) box — but the outgoing label (idleFadeAnimCb) stays visibly
  // on screen, fading out over IDLE_FADE_MS, at this *previous* position for
  // a while after posX/posY above has already moved on to the new spot. A
  // fresh transition that only avoided the other element's current box could
  // still land right on top of its still-fading-out old one — the real
  // remaining gap behind T43's "rare but real" overlap reports. Valid (still
  // worth checking) only while millis() < fadeOutUntilMs.
  int16_t prevPosX, prevPosY;
  int16_t prevHalfW, prevHalfH;
  uint32_t fadeOutUntilMs;
};
IdleElement idleTimeElement;
IdleElement idleDateElement;

// Font pool: every Montserrat size already enabled in lv_conf.h for other
// screens (see each one's own bundled-font comment there) — no new font
// assets needed. T42: dropped 22/24 (Brian: too small to read comfortably at
// a glance). Deliberately not trying to add real bold/regular *weight*
// variants (LVGL's bundled fonts are one weight per size) — varying size
// already changes each glyph's actual lit pixels meaningfully, which is the
// part that matters here.
constexpr const lv_font_t *IDLE_FONT_POOL[] = {
    &lv_font_montserrat_28, &lv_font_montserrat_42, &lv_font_montserrat_48};
constexpr size_t IDLE_FONT_POOL_COUNT = sizeof(IDLE_FONT_POOL) / sizeof(IDLE_FONT_POOL[0]);
// T43: replaced T42's fixed IDLE_POSITION_POOL with real geometry —
// pickIdlePosition() rejection-samples within this radius and validates the
// *actual* measured text box against it, rather than trusting a hand-picked
// offset to be safe for every font/string combo.
constexpr int16_t IDLE_PANEL_RADIUS = 195; // 2.04" round 390x390 panel (HARDWARE.md), diameter/2
constexpr int16_t IDLE_SAFE_RADIUS = 185;  // small inward margin from the true bezel edge
constexpr int16_t IDLE_MIN_REPEAT_DISTANCE_PX = 50; // must move at least this far from its own last spot (bug #3)
constexpr int16_t IDLE_MIN_ELEMENT_GAP_PX = 10;     // required clear gap between the two elements' boxes (bug #2)
// T50: single sample budget for pickIdlePosition()'s best-effort search
// (replaced the old fixed 3-tier x 40-attempt scheme) — see that function's
// own comment for why.
constexpr int IDLE_POSITION_ATTEMPTS = 60;
// Randomized per-cycle rather than a flat interval — each element re-rolls
// its own dwell independently every transition, which is what actually keeps
// date and time drifting out of phase with each other rather than sharing
// one synchronized cadence (a fixed interval for both would either stay in
// permanent lockstep if started together, or at best hold one fixed phase
// offset forever). T45 (2026-08-26): roughly doubled from T41/T42's 9-15s —
// Brian wanted the cycle slower after living with it, no exact target given
// (same "pick something reasonable, he'll iterate live" pattern as T41's
// original cycle-slowdown ask).
constexpr uint32_t IDLE_DWELL_MIN_MS = 20000;
constexpr uint32_t IDLE_DWELL_MAX_MS = 30000;
constexpr uint32_t IDLE_FADE_MS = 1500; // cross-fade duration — unchanged from T41, "~1-2s" per Brian

void idleFadeAnimCb(void *obj, int32_t value) {
  lv_obj_set_style_text_opa((lv_obj_t *)obj, (lv_opa_t)value, 0);
}

// Does the (cx,cy)-centered box with the given half-extents stay fully
// inside the panel's visible circle? Checks all 4 corners, not just the
// center point — the fix for bug #1 (T43): a box can be well inside the
// square coordinate frame LVGL positions against and still clip against the
// physical round bezel once the text's real width/height are accounted for.
bool idleBoxFitsCircle(int16_t cx, int16_t cy, int16_t halfW, int16_t halfH, int16_t radius) {
  for (int8_t sx = -1; sx <= 1; sx += 2) {
    for (int8_t sy = -1; sy <= 1; sy += 2) {
      int32_t px = cx + sx * halfW;
      int32_t py = cy + sy * halfH;
      if (px * px + py * py > (int32_t)radius * radius) return false;
    }
  }
  return true;
}

// Signed clearance between two boxes: positive = genuinely separated by
// this many px (the real gap on whichever axis is doing the separating);
// negative = overlapping, magnitude = the shallower (smaller) of the two
// axes' penetration depth — i.e. how far the box would need to move to
// just barely stop overlapping. Replaces T43/T47's boolean
// idleBoxesTooClose() — T50 needs an actual *measure* of how bad a
// candidate is, not just pass/fail, to rank fallback candidates (see
// pickIdlePosition()).
int32_t idleBoxClearance(int16_t ax, int16_t ay, int16_t aHalfW, int16_t aHalfH, int16_t bx, int16_t by,
                          int16_t bHalfW, int16_t bHalfH) {
  int32_t xGap = (int32_t)max(ax - aHalfW, bx - bHalfW) - (int32_t)min(ax + aHalfW, bx + bHalfW);
  int32_t yGap = (int32_t)max(ay - aHalfH, by - bHalfH) - (int32_t)min(ay + aHalfH, by + bHalfH);
  return xGap > yGap ? xGap : yGap;
}

// T51 (2026-08-26): now returns whether the candidate actually found is
// *perfect* (repeat-safe and gap-clear) or only best-effort — see
// pickIdleFontAndPosition() below, which uses this to retry the whole
// search at a smaller font size instead of accepting a compromised
// placement outright, per Brian's own explicit framing.
//
// T50 (2026-08-26): Brian kept seeing a plain, direct overlap — one element
// fading in right on top of the other's *current* box — even after T47/T49.
// Root cause, confirmed against this font's real bundled glyph-width table
// (not a guess): at 48pt, the date string ("Wed Aug 26") measures roughly
// 300px wide. For a box that size to pass idleBoxFitsCircle() against
// IDLE_SAFE_RADIUS (185px), its center has to land within roughly a 30px
// radius of the panel's own center — a *small* valid region. Time's own
// valid region at the same font size is much larger (its string is
// shorter), and often sits centrally too. When both land near center at
// once, all `IDLE_POSITION_ATTEMPTS` tries can genuinely fail to find any
// spot that's both circle-safe *and* clear of the other element — a real,
// not rare, geometric conflict, not just occasional bad luck.
//
// T43/T47's old fix (3 tiers, each dropping a constraint, falling through
// to "accept the first fit-only random point") handled that failure badly:
// tier 2 would accept *any* circle-safe point with zero regard for how
// close it was to the other element — including landing exactly on top of
// it, which is exactly what Brian saw. Replaced with a single best-effort
// search: every circle-fitting candidate sampled is scored by
// idleBoxClearance() against the other element's box (and its still-fading
// -out previous box, if any — see IdleElement's own comment), and the
// *best-scoring* one found is used if no perfect (repeat-safe and
// gap-clear) candidate turns up. This can't fully eliminate overlap in the
// genuinely-tight case above (there may be no candidate with real
// clearance at all), but it guarantees the *least bad* option found is
// used, never an arbitrary/unconstrained one — this project's floor is now
// "may still overlap a little in a tight case," not "may land exactly on
// top." circle-fit itself is still never relaxed (bug #1 stays fixed
// unconditionally). `debugLabel` ("time"/"date") is only used for the
// WebSerial line below, printed only when a real conflict forced a
// best-effort (non-ideal) pick — a future capture during a repro would
// show exactly how often/tight this geometric conflict actually bites.
bool pickIdlePosition(int16_t halfW, int16_t halfH, const IdleElement &self, const IdleElement &other,
                       const char *debugLabel, int16_t *outX, int16_t *outY) {
  bool otherFadingOut = other.hasPosition && millis() < other.fadeOutUntilMs;

  int16_t bestX = 0, bestY = 0;
  int32_t bestClearance = INT32_MIN;
  bool bestSatisfiesRepeat = false;
  bool haveCandidate = false;

  for (int attempt = 0; attempt < IDLE_POSITION_ATTEMPTS; attempt++) {
    int16_t cx = (int16_t)random(-IDLE_SAFE_RADIUS, IDLE_SAFE_RADIUS + 1);
    int16_t cy = (int16_t)random(-IDLE_SAFE_RADIUS, IDLE_SAFE_RADIUS + 1);
    if (!idleBoxFitsCircle(cx, cy, halfW, halfH, IDLE_SAFE_RADIUS)) continue; // hard constraint, never relaxed

    bool satisfiesRepeat = !self.hasPosition;
    if (self.hasPosition) {
      int32_t dx = cx - self.posX, dy = cy - self.posY;
      satisfiesRepeat = dx * dx + dy * dy >= (int32_t)IDLE_MIN_REPEAT_DISTANCE_PX * IDLE_MIN_REPEAT_DISTANCE_PX;
    }

    int32_t clearance = INT32_MAX; // nothing to conflict with yet (shouldn't happen post-T49, but stay safe)
    if (other.hasPosition) {
      clearance = idleBoxClearance(cx, cy, halfW, halfH, other.posX, other.posY, other.halfW, other.halfH);
      if (otherFadingOut) {
        int32_t clearancePrev = idleBoxClearance(cx, cy, halfW, halfH, other.prevPosX, other.prevPosY,
                                                  other.prevHalfW, other.prevHalfH);
        clearance = clearance < clearancePrev ? clearance : clearancePrev; // must clear *both* boxes other may show
      }
    }

    // Perfect candidate — clears the real gap requirement against `other`
    // and doesn't repeat `self`'s own last spot. Take it immediately, no
    // need to keep sampling.
    if (satisfiesRepeat && clearance >= IDLE_MIN_ELEMENT_GAP_PX) {
      *outX = cx;
      *outY = cy;
      return true;
    }

    // Otherwise, remember this candidate if it's the best seen so far.
    // Clearance from the other element is the primary sort key (landing on
    // top of *different* text is the actually-illegible failure Brian
    // reported); the repeat-distance miss is only a tie-breaker (re-landing
    // near this same element's own last spot is a duller failure).
    bool better = !haveCandidate || clearance > bestClearance ||
                  (clearance == bestClearance && satisfiesRepeat && !bestSatisfiesRepeat);
    if (better) {
      bestX = cx;
      bestY = cy;
      bestClearance = clearance;
      bestSatisfiesRepeat = satisfiesRepeat;
      haveCandidate = true;
    }
  }

  if (haveCandidate) {
    if (bestClearance < IDLE_MIN_ELEMENT_GAP_PX) {
      WebSerial.printf("idlePosition: t=%ums element=%s best-effort clearance=%dpx (wanted >=%dpx)\n", millis(),
                        debugLabel, (int)bestClearance, IDLE_MIN_ELEMENT_GAP_PX);
    }
    *outX = bestX;
    *outY = bestY;
    return false;
  }

  // Only reachable if literally nothing sampled fit the circle at all —
  // shouldn't happen for any font/string this project's IDLE_FONT_POOL
  // produces, but a safe last resort regardless.
  WebSerial.printf("idlePosition: t=%ums element=%s found nothing that fits the circle, using center fallback\n",
                    millis(), debugLabel);
  *outX = 0;
  *outY = 0;
  return false;
}

// T51 (2026-08-26), Brian's own explicit framing: when a cycle's font
// choice can't find a fully clear position, "make sure there is nothing in
// the space it would need to occupy and if so, move to the next option ...
// if not, then yes, eliminate the [larger font] option" — i.e. don't just
// accept T50's best-effort compromise at the size that was randomly picked;
// retry the whole search one font size smaller first, since a smaller box
// is intrinsically easier to place clear of the other element (see T50's
// own comment on why — a smaller box's circle-safe region is larger). Only
// once even the pool's smallest font can't find a clear spot does a
// best-effort (possibly still slightly overlapping) placement get accepted
// — the true last resort, not the first move.
//
// Picks this cycle's starting font at random same as before, then walks
// *down* through the pool (IDLE_FONT_POOL is smallest-to-largest, so this
// walks the array backward from the random start) trying each size in
// turn. Returns the font actually used; fills *outX/*outY with its position
// and *outHalfW/*outHalfH with that font's own measured box (so the caller
// doesn't need to re-measure — this function already did, for whichever
// font actually won).
const lv_font_t *pickIdleFontAndPosition(const char *text, const IdleElement &self, const IdleElement &other,
                                          const char *debugLabel, int16_t *outX, int16_t *outY, int16_t *outHalfW,
                                          int16_t *outHalfH) {
  size_t startIndex = random(IDLE_FONT_POOL_COUNT);

  const lv_font_t *fallbackFont = IDLE_FONT_POOL[0];
  int16_t fallbackX = 0, fallbackY = 0;
  int16_t fallbackHalfW = 0, fallbackHalfH = 0;

  for (size_t i = startIndex + 1; i-- > 0;) { // counts down from startIndex to 0 inclusive
    const lv_font_t *font = IDLE_FONT_POOL[i];
    lv_point_t textSize;
    // T43: measure the *actual* rendered box for this candidate font+string
    // (not a hand-picked guess) — see this project's own comment further up
    // on why 0/0 letter/line-space and the +2px padding match how these
    // labels actually render.
    lv_text_get_size(&textSize, text, font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    int16_t halfW = (int16_t)(textSize.x / 2) + 2;
    int16_t halfH = (int16_t)(textSize.y / 2) + 2;

    int16_t cx, cy;
    bool perfect = pickIdlePosition(halfW, halfH, self, other, debugLabel, &cx, &cy);
    if (perfect) {
      *outX = cx;
      *outY = cy;
      *outHalfW = halfW;
      *outHalfH = halfH;
      return font;
    }
    // Keep the smallest font tried so far as the running fallback — smaller
    // sizes are intrinsically at least as easy to place as larger ones
    // already tried, so the last (smallest) one reached below is the best
    // fallback available.
    fallbackFont = font;
    fallbackX = cx;
    fallbackY = cy;
    fallbackHalfW = halfW;
    fallbackHalfH = halfH;
  }

  // Only reached if not even the pool's smallest font found a fully clear
  // spot (the loop above always runs at least once, so the fallback* values
  // are always real by now) — the genuinely-tight case T50 first
  // identified, now only accepted after every smaller option has already
  // failed too.
  WebSerial.printf(
      "idlePosition: t=%ums element=%s no font size in the pool found a fully clear spot, using smallest-tried\n",
      millis(), debugLabel);
  *outX = fallbackX;
  *outY = fallbackY;
  *outHalfW = fallbackHalfW;
  *outHalfH = fallbackHalfH;
  return fallbackFont;
}

// Mid-grey, not full white — a slightly muted text color reads more
// deliberately "ambient" than stark white against pure black.
lv_obj_t *buildIdleLabel(lv_obj_t *scr, const char *initialText, bool visible) {
  lv_obj_t *label = lv_label_create(scr);
  lv_obj_set_style_text_font(label, IDLE_FONT_POOL[0], 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0x808080), 0);
  lv_obj_center(label);
  lv_label_set_text(label, initialText);
  lv_obj_set_style_text_opa(label, visible ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
  return label;
}

// T49 (2026-08-26): fills in an IdleElement's box fields for its *initial*
// build-time placeholder (before it has ever gone through a real
// pickIdlePosition() transition) — see buildIdleScreen()'s own comment for
// why this has to be a real, accurate box rather than leaving hasPosition
// false. Measured the same way updateIdleElement() measures a real cycle's
// box, for the same font/text actually shown.
void setIdleElementInitialBox(IdleElement &el, const char *initialText) {
  lv_point_t sz;
  lv_text_get_size(&sz, initialText, IDLE_FONT_POOL[0], 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
  el.posX = 0;
  el.posY = 0;
  el.halfW = (int16_t)(sz.x / 2) + 2;
  el.halfH = (int16_t)(sz.y / 2) + 2;
  el.hasPosition = true;
}

void buildIdleScreen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  idleTimeElement.labelA = buildIdleLabel(scr, "--:--", true); // placeholder until updateIdleScreen()'s first real call
  idleTimeElement.labelB = buildIdleLabel(scr, "", false);     // hidden until the first cross-fade brings it in
  idleTimeElement.aIsVisible = true;
  idleTimeElement.lastTransitionMs = 0;
  idleTimeElement.dwellMs = (uint32_t)random(IDLE_DWELL_MIN_MS, IDLE_DWELL_MAX_MS + 1);
  idleTimeElement.fadeOutUntilMs = 0;
  // T49: root cause of "one element fades in directly on top of the other"
  // — hasPosition used to start false for *both* elements, so whichever one
  // transitions first (a coin flip: each element's very first dwell is
  // independently randomized, nothing orders them) saw `other.hasPosition ==
  // false` and skipped the overlap check entirely, landing anywhere,
  // including right on the other element's still-visible build-time
  // placeholder. Since this only takes one element's-worth of bad luck per
  // boot, and this project reboots on every single flash, it was hitting
  // roughly every other boot — not a subtle mid-transition timing edge case
  // at all, just a plain missing initial value. Fixed by seeding a real,
  // accurate initial box for both elements right here instead of leaving
  // hasPosition false until their first real transition.
  setIdleElementInitialBox(idleTimeElement, "--:--");

  idleDateElement.labelA = buildIdleLabel(scr, "", true); // blank (not a fake date) until first real call
  idleDateElement.labelB = buildIdleLabel(scr, "", false);
  idleDateElement.aIsVisible = true;
  idleDateElement.lastTransitionMs = 0;
  idleDateElement.dwellMs = (uint32_t)random(IDLE_DWELL_MIN_MS, IDLE_DWELL_MAX_MS + 1);
  idleDateElement.fadeOutUntilMs = 0;
  // Blank initial text -> a real but zero-size box (nothing to actually
  // block against) — same helper, kept symmetric with time's above rather
  // than special-casing "empty means skip."
  setIdleElementInitialBox(idleDateElement, "");
}

// Fills *outTm with this device's current local time (UTC + utcOffsetMinutes,
// see that global's own top-of-file comment) and returns true, or returns
// false if SNTP hasn't synced yet. Shared by updateIdleElement() and the
// /debug/time HTTP route (setupHttpOta()) so the same computation can be
// confirmed remotely without needing physical access to the round panel.
//
// getLocalTime(..., 0): a single non-blocking check, never waits — loop()
// must never block (see the "Docker app sync protocol" section's top
// comment for why). Returns false until SNTP has actually synced at least
// once (see setupTimeSync()), e.g. briefly after boot before WiFi finishes
// connecting. Only used here as that sync-ready check, though — the
// `timeinfo` it fills is discarded and re-derived below (T44).
bool getIdleLocalTime(struct tm *outTm) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) return false;
  // T44 (2026-08-26): local time is now PC-sourced (utcOffsetMinutes, via
  // pollSyncState()/pcState.utcOffsetMinutes), not a hardcoded POSIX-TZ
  // string — setupTimeSync() fixes this device's own clock to plain UTC, so
  // this manual epoch shift is the *only* "local time" conversion that
  // happens anywhere on this device. gmtime_r (not localtime_r) on the
  // shifted epoch — using localtime_r again would silently reapply whatever
  // TZ this device's C library thinks it has (harmless today since that's
  // fixed to UTC0, but conceptually wrong: the offset already *is* the
  // local-time conversion).
  time_t nowUtc;
  time(&nowUtc);
  time_t localEpoch = nowUtc + (time_t)utcOffsetMinutes * 60;
  gmtime_r(&localEpoch, outTm);
  return true;
}

// One element's (time or date) own transition check/cross-fade — see
// IdleElement's own comment for why each needs two labels. `other` is the
// sibling element (time passes date and vice versa), read-only here, so this
// element's new position can avoid landing on top of wherever the other one
// currently is (bug #2). Cheap to call unconditionally, since it only
// actually does anything once per this element's own dwellMs, not every
// call.
void updateIdleElement(IdleElement &el, const IdleElement &other, bool isTime) {
  uint32_t now = millis();
  if (el.lastTransitionMs == 0) {
    el.lastTransitionMs = now; // first call ever — just start this element's own dwell clock, nothing to animate yet
    return;
  }
  if (now - el.lastTransitionMs < el.dwellMs) return;
  el.lastTransitionMs = now;
  el.dwellMs = (uint32_t)random(IDLE_DWELL_MIN_MS, IDLE_DWELL_MAX_MS + 1); // re-roll for next cycle

  lv_obj_t *outgoing = el.aIsVisible ? el.labelA : el.labelB;
  lv_obj_t *incoming = el.aIsVisible ? el.labelB : el.labelA;
  el.aIsVisible = !el.aIsVisible;

  struct tm timeinfo;
  char buf[24];
  if (getIdleLocalTime(&timeinfo)) {
    // Standard (non-GNU) strftime specifiers only — the toolchain's libc
    // here is a trimmed embedded newlib, not glibc, so the "%-I"/"%-d"
    // no-leading-zero GNU extensions aren't something to rely on.
    strftime(buf, sizeof(buf), isTime ? "%I:%M %p" : "%a %b %d", &timeinfo);
  } else {
    strcpy(buf, isTime ? "--:--" : "");
  }
  lv_label_set_text(incoming, buf);

  // T51: font and position are now chosen together — if the (randomly
  // started) font size can't find a fully clear spot, this retries the
  // whole search one size smaller rather than accepting a compromised
  // placement at the original size. See pickIdleFontAndPosition()'s own
  // comment.
  int16_t posX, posY, halfW, halfH;
  const lv_font_t *font =
      pickIdleFontAndPosition(buf, el, other, isTime ? "time" : "date", &posX, &posY, &halfW, &halfH);
  lv_obj_set_style_text_font(incoming, font, 0);
  lv_obj_align(incoming, LV_ALIGN_CENTER, posX, posY);

  // T47: stash the box we're vacating *before* overwriting it — its label
  // (outgoing, above) keeps fading out here for IDLE_FADE_MS, so the other
  // element still needs to be able to see and avoid it for that long. Only
  // meaningful if there was a previous real position (hasPosition) — the
  // very first transition has no old box to protect.
  if (el.hasPosition) {
    el.prevPosX = el.posX;
    el.prevPosY = el.posY;
    el.prevHalfW = el.halfW;
    el.prevHalfH = el.halfH;
    el.fadeOutUntilMs = now + IDLE_FADE_MS;
  }
  el.posX = posX;
  el.posY = posY;
  el.halfW = halfW;
  el.halfH = halfH;
  el.hasPosition = true;

  lv_anim_t fadeOut;
  lv_anim_init(&fadeOut);
  lv_anim_set_var(&fadeOut, outgoing);
  lv_anim_set_exec_cb(&fadeOut, idleFadeAnimCb);
  lv_anim_set_values(&fadeOut, LV_OPA_COVER, LV_OPA_TRANSP);
  lv_anim_set_time(&fadeOut, IDLE_FADE_MS);
  lv_anim_start(&fadeOut);

  lv_anim_t fadeIn;
  lv_anim_init(&fadeIn);
  lv_anim_set_var(&fadeIn, incoming);
  lv_anim_set_exec_cb(&fadeIn, idleFadeAnimCb);
  lv_anim_set_values(&fadeIn, LV_OPA_TRANSP, LV_OPA_COVER);
  lv_anim_set_time(&fadeIn, IDLE_FADE_MS);
  lv_anim_start(&fadeIn);
}

// Called once right before this screen is shown, and every loop() pass while
// it stays shown. Time and date transition completely independently of each
// other — see updateIdleElement()/IDLE_DWELL_MIN_MS/MAX_MS.
void updateIdleScreen() {
  updateIdleElement(idleTimeElement, idleDateElement, true);
  updateIdleElement(idleDateElement, idleTimeElement, false);
}
