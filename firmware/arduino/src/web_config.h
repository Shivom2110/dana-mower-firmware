#pragma once

#include <stdint.h>

// Wi-Fi web-control env (giga_r1_m7_web). Settings for the GIGA's Wi-Fi link
// to the SmartMow website in /website.

#if __has_include("web_secrets.h")
#include "web_secrets.h"
#endif

// true:  the GIGA creates its own Wi-Fi network (no router needed). Join it
//        from the laptop running the website; the GIGA's IP is printed on
//        the serial monitor (typically 192.168.3.1).
// false: the GIGA joins an existing network (WEB_STA_SSID / WEB_STA_PASS);
//        its DHCP address is printed on the serial monitor.
#ifndef WEB_WIFI_AP_MODE
#define WEB_WIFI_AP_MODE true
#endif

// Access-point credentials. WPA2 needs a password of at least 8 characters.
// Change these before using the mower anywhere other people can join.
#define WEB_AP_SSID "SmartMow-GIGA"
#define WEB_AP_PASS "smartmow2026"

// Station-mode credentials (only used when WEB_WIFI_AP_MODE is false).
// Put real ones in web_secrets.h (git-ignored), for example:
//   #define WEB_WIFI_AP_MODE false
//   #define WEB_STA_SSID "My Phone"
//   #define WEB_STA_PASS "hotspot-password"
#ifndef WEB_STA_SSID
#define WEB_STA_SSID ""
#endif
#ifndef WEB_STA_PASS
#define WEB_STA_PASS ""
#endif

static constexpr uint16_t WEB_HTTP_PORT = 80;

// Station mode: how often loop() retries joining after the network is lost.
static constexpr uint32_t WEB_STA_RETRY_MS = 10000;

// Dead-man timeout: the website re-sends /drive every 100 ms while a drive
// button is held. If no /drive arrives for this long, the wheels ramp to
// 0 rpm and stop (brake engages through the inverter).
static constexpr uint32_t WEB_COMMAND_TIMEOUT_MS = 500;

// An HTTP client that has not sent a complete request by then is dropped,
// so one slow connection cannot hold the server.
static constexpr uint32_t WEB_CLIENT_TIMEOUT_MS = 300;

// Wheel command ramp, in full-scale units per second (same as main.cpp).
static constexpr float WEB_SLEW_PER_SEC = 2.0f;
