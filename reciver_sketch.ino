#include <WiFi.h>

const char* BEACON_SSIDS[] = {"beacon_1", "beacon_2", "beacon_3"};
const int NUM_BEACONS = 3;

// Координаты маяков в метрах (x, y)
float beacon_coords[NUM_BEACONS][2] = {
  {0.6, 0.6},  // Координаты beacon_1
  {0.0, 0.0},  // Координаты beacon_2
  {1.2, 0.0}   // Координаты beacon_3
};

// Калибровочные параметры для формулы RSSI -> расстояние
const float A = -61.5; // Мощность сигнала на расстоянии 1 метра
const float n = 2.5; // Коэффициент затухания сигнала

float distances[NUM_BEACONS];

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Receiver setup complete.");
}

void loop() {
  Serial.println("Scanning for Wi-Fi networks...");
  int networksFound = WiFi.scanNetworks();

  if (networksFound == 0) {
    Serial.println("No networks found.");
  } else {
    int beacons_found_count = 0;
    for (int i = 0; i < networksFound; i++) {
      String current_ssid = WiFi.SSID(i);
      int current_rssi = WiFi.RSSI(i);

      for (int j = 0; j < NUM_BEACONS; j++) {
        if (current_ssid == BEACON_SSIDS[j]) {
          distances[j] = calculateDistance(current_rssi);
          Serial.printf("Found %s (RSSI: %d dBm) -> Distance: %.2f m\n", BEACON_SSIDS[j], current_rssi, distances[j]);
          beacons_found_count++;
          break;
        }
      }
    }

    if (beacons_found_count == NUM_BEACONS) {
      float x, y;
      trilaterate(distances[0], distances[1], distances[2], &x, &y);
      Serial.printf("\n>>> Calculated Position: X = %.2f, Y = %.2f <<<\n\n", x, y);
    } else {
      Serial.println("Could not find all beacons to perform trilateration.\n");
    }
  }
  
  delay(1000);
}

// Функция для преобразования RSSI в расстояние
float calculateDistance(int rssi) {
  return pow(10.0, (A - rssi) / (10.0 * n));
}

// Функция для трилатерации
void trilaterate(float r1, float r2, float r3, float* x, float* y) {
  float x1 = beacon_coords[0][0];
  float y1 = beacon_coords[0][1];
  float x2 = beacon_coords[1][0];
  float y2 = beacon_coords[1][1];
  float x3 = beacon_coords[2][0];
  float y3 = beacon_coords[2][1];

  float A_tri = 2 * x2 - 2 * x1;
  float B_tri = 2 * y2 - 2 * y1;
  float C_tri = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
  float D_tri = 2 * x3 - 2 * x2;
  float E_tri = 2 * y3 - 2 * y2;
  float F_tri = r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3;
  
  *x = (C_tri * E_tri - F_tri * B_tri) / (E_tri * A_tri - B_tri * D_tri);
  *y = (C_tri * D_tri - F_tri * A_tri) / (B_tri * D_tri - A_tri * E_tri);
}
