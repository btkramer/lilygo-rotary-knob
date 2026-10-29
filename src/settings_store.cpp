// settings_store.cpp — NVS persistence of synced settings and sync-task startup.

#include <Arduino.h>
#include <Preferences.h>
#include "buzzer.h"
#include "screen_nav.h"
#include "screen_home.h"
#include "screen_volume.h"
#include "hub_sync.h"
#include "settings_store.h"

Preferences syncPrefs; // only ever touched from syncPollTask post-refactor — see setupSyncSettings()'s comment

// Loads NVS-persisted settings (if any) over the hardcoded defaults already
// assigned at global scope, then starts the background poll task. Doesn't
// need WiFi first — called early in setup(), before setupOTA() connects —
// operator-adjusted values (from a previous sync) take effect even if this
// boot never reaches the Docker app at all, and the task itself just finds
// WiFi not-yet-connected and retries at the fast interval until it is (same
// as any other disconnected state, no special-casing needed). Pinned to
// core 0 to stay clear of core 1's loop()/LVGL rendering entirely — not
// just avoiding blocking it, but not competing with it for CPU time either.
// syncPrefs (NVS) is opened here, on the main task, then only ever written
// from syncPollTask afterward — sequential-but-cross-task, never
// concurrent, since this function returns (and its effects are published to
// the new task via xTaskCreatePinnedToCore) before the task's first poll
// could possibly reach a putUInt() call.
void setupSyncSettings() {
  syncPrefs.begin("sync", false);
  volumeFuseMs = syncPrefs.getUInt("volFuseMs", VOLUME_FUSE_MS_DEFAULT);
  idleToHomeMs = syncPrefs.getUInt("idleMs", IDLE_TO_HOME_MS_DEFAULT);
  dimDelayMs = syncPrefs.getUInt("dimDelayMs", DIM_DELAY_MS_DEFAULT);
  utcOffsetMinutes = syncPrefs.getInt("utcOffMin", 0); // T44 — 0 (plain UTC) until a real value has ever synced
  buzzerEnabled = syncPrefs.getBool("buzzerEn", true); // T60 — on by default until a real value has ever synced
  attentionRenagMinutes = syncPrefs.getUInt("renagMin", ATTENTION_RENAG_MINUTES_DEFAULT); // T89
  Serial.printf("sync: loaded from NVS: volumeFuseMs=%u idleToHomeMs=%u dimDelayMs=%u utcOffsetMinutes=%d buzzerEnabled=%d attentionRenagMinutes=%u\n",
                volumeFuseMs, idleToHomeMs, dimDelayMs, utcOffsetMinutes, buzzerEnabled, attentionRenagMinutes);

  syncResultMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(syncPollTask, "syncPoll", 8192, NULL, 1, NULL, 0);
}
