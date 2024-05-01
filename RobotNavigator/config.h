#define SECRET_SSID "rob011235"
#define SECRET_PASS "5235235235"

#define SPIWIFI       SPI
#define SPIWIFI_SS    10   // Chip select pin
#define SPIWIFI_ACK    9   // a.k.a BUSY or READY pin
#define ESP32_RESETN   6  // Reset pin
#define ESP32_GPIO0   -1   // Not connected

struct coord{
  double lat;
  double lon;
  double alt;
};