
#include "esp_mesh.h"



// Callback functions for mesh events
void meshEventCallback(mesh_event_t event);
void meshStationConnectedCallback(mesh_addr_t mesh_addr);
void meshStationDisconnectedCallback(mesh_addr_t mesh_addr);

void setup() {
  Serial.begin(115200);



  // Initialize mesh networking
  esp_err_t err = esp_mesh_init();
  if (err != ESP_OK) {
    Serial.printf("Mesh init failed: %d\n", err);
    return;
  }

  // Set the mesh configurations
  mesh_cfg_t mesh_config = MESH_INIT_CONFIG_DEFAULT();
  /*
  mesh_config.channel = MESH_CHANNEL;
  mesh_config.router.ssid = MESH_SSID;
  mesh_config.router.password = MESH_PASSWORD;
  mesh_config.router.duty_cycle = 50;
  mesh_config.router.ap_authmode = WIFI_AUTH_WPA2_PSK;
  mesh_config.router.max_connection = MESH_MAX_CONNECTIONS;
*/

  // Set the mesh event callbacks
  esp_mesh_set_topology(MESH_TOPO_CHAIN);
  esp_mesh_set_self_organized(true, MESH_ROOT);
  esp_mesh_set_event_callback(meshEventCallback);

  // Start the mesh networking
  err = esp_mesh_set_config(&mesh_config);
  if (err != ESP_OK) {
    Serial.printf("Mesh set config failed: %d\n", err);
    return;
  }

  // Wait for mesh network to be established
  while (!esp_mesh_is_root()) {
    delay(1000);
    Serial.println("Waiting for mesh network...");
  }
  Serial.println("Mesh network established.");
}

void loop() {
  // Your main loop code here
}

void meshEventCallback(mesh_event_t event) {
  switch (event.id) {
    case MESH_EVENT_STARTED:
      Serial.println("Mesh network started.");
      break;
    case MESH_EVENT_STOPPED:
      Serial.println("Mesh network stopped.");
      break;
    case MESH_EVENT_CHILD_CONNECTED:
      Serial.println("New station connected to the mesh.");
      break;
    case MESH_EVENT_CHILD_DISCONNECTED:
      Serial.println("Station disconnected from the mesh.");
      break;
    default:
      break;
  }
}

void meshStationConnectedCallback(mesh_addr_t mesh_addr) {
  Serial.printf("Station connected: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n", 
                mesh_addr.addr[0], mesh_addr.addr[1], mesh_addr.addr[2], 
                mesh_addr.addr[3], mesh_addr.addr[4], mesh_addr.addr[5]);
}

void meshStationDisconnectedCallback(mesh_addr_t mesh_addr) {
  Serial.printf("Station disconnected: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n", 
                mesh_addr.addr[0], mesh_addr.addr[1], mesh_addr.addr[2], 
                mesh_addr.addr[3], mesh_addr.addr[4], mesh_addr.addr[5]);
}
