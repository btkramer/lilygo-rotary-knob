// settings_store.h — NVS persistence of synced settings and sync-task startup.
#pragma once

#include <Arduino.h>
#include <Preferences.h>

extern Preferences syncPrefs;
void setupSyncSettings();
