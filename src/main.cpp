#include <Arduino.h>
#include "Matter.h"
#include <app/server/OnboardingCodesUtil.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
using namespace chip;
using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::endpoint;
#include <SPI.h>
#include <Wire.h>
#include <PCA9685.h>
#include <ColorConverter.h>

PCA9685 pwm;

// LED functions
void setLEDs(byte channel, uint16_t h, uint16_t s, uint16_t v);
void ledHandler();
void sunHandler();
void stormHandler();

uint16_t fadeInterval = 100;
bool countingUp = false;
uint16_t saturationValue = 100;
uint32_t lastLedUpdate;
uint32_t stormInterval = 2000;
bool isLightning = false;
#define HUE_SUN 30
#define HUE_STORM 230

enum LightMode {
  SUN,
  STORM
};
uint8_t lightMode = SUN;

// Please configure your PINs
const int LED_PIN = 2;
const int TOGGLE_BUTTON_PIN = 0;

// Debounce for toggle button
const int DEBOUNCE_DELAY = 500;
int last_toggle;

// Cluster and attribute ID used by Matter light device
const uint32_t CLUSTER_ID = OnOff::Id;
const uint32_t ATTRIBUTE_ID = OnOff::Attributes::OnOff::Id;

// Endpoint and attribute ref that will be assigned to Matter device
uint16_t light_endpoint_id = 0;
attribute_t *attribute_ref;

// There is possibility to listen for various device events, related for example to setup process
// Leaved as empty for simplicity
static void on_device_event(const ChipDeviceEvent *event, intptr_t arg) {}
static esp_err_t on_identification(identification::callback_type_t type, uint16_t endpoint_id,
                                   uint8_t effect_id, uint8_t effect_variant, void *priv_data) {
  return ESP_OK;
}

// Listener on attribute update requests.
// In this example, when update is requested, path (endpoint, cluster and attribute) is checked
// if it matches light attribute. If yes, LED changes state to new one.
static esp_err_t on_attribute_update(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                     uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
  if (type == attribute::PRE_UPDATE && endpoint_id == light_endpoint_id &&
      cluster_id == CLUSTER_ID && attribute_id == ATTRIBUTE_ID) {
    // We got an light on/off attribute update!
    bool new_state = val->val.b;
    if (new_state) {
      lightMode = STORM;
    } 
    else {
      lightMode = SUN;
    }
    digitalWrite(LED_PIN, new_state);
  }
  return ESP_OK;
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(TOGGLE_BUTTON_PIN, INPUT);

  // Enable debug logging
  esp_log_level_set("*", ESP_LOG_DEBUG);

  // Setup Matter node
  node::config_t node_config;
  node_t *node = node::create(&node_config, on_attribute_update, on_identification);

  // Setup Light endpoint / cluster / attributes with default values
  on_off_light::config_t light_config;
  light_config.on_off.on_off = false;
  light_config.on_off.lighting.start_up_on_off = false;
  endpoint_t *endpoint = on_off_light::create(node, &light_config, ENDPOINT_FLAG_NONE, NULL);

  // Save on/off attribute reference. It will be used to read attribute value later.
  attribute_ref = attribute::get(cluster::get(endpoint, CLUSTER_ID), ATTRIBUTE_ID);

  // Save generated endpoint id
  light_endpoint_id = endpoint::get_id(endpoint);
  
  // Setup DAC (this is good place to also set custom commission data, passcodes etc.)
  esp_matter::set_custom_dac_provider(chip::Credentials::Examples::GetExampleDACProvider());

  // Start Matter device
  esp_matter::start(on_device_event);

  // Print codes needed to setup Matter device
  PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

  // pwm.begin();
  // pwm.setOscillatorFrequency(27000000);
  // pwm.setPWMFreq(1000);  // This is the maximum PWM frequency
  // Wire.setClock(400000);
  Wire.begin();
  Wire.setClock(400000);
  pwm.resetDevices();
  pwm.init();
  pwm.setPWMFrequency(1000);
}

// Reads light on/off attribute value
esp_matter_attr_val_t get_onoff_attribute_value() {
  esp_matter_attr_val_t onoff_value = esp_matter_invalid(NULL);
  attribute::get_val(attribute_ref, &onoff_value);
  return onoff_value;
}

// Sets light on/off attribute value
void set_onoff_attribute_value(esp_matter_attr_val_t* onoff_value) {
  attribute::update(light_endpoint_id, CLUSTER_ID, ATTRIBUTE_ID, onoff_value);
}

// When toggle light button is pressed (with debouncing),
// light attribute value is changed
void loop() {
  if ((millis() - last_toggle) > DEBOUNCE_DELAY) {
    if (!digitalRead(TOGGLE_BUTTON_PIN)) {
      last_toggle = millis();
      // Read actual on/off value, invert it and set
      esp_matter_attr_val_t onoff_value = get_onoff_attribute_value();
      onoff_value.val.b = !onoff_value.val.b;
      set_onoff_attribute_value(&onoff_value);
    }
  }
  ledHandler();
}

void ledHandler() {
  if (lightMode == SUN) {
    sunHandler();
  }
  else if (lightMode == STORM) {
    stormHandler();
  }
}

// Shift between overcast and sun
void sunHandler() {
  if (millis() - lastLedUpdate >= fadeInterval) {
    lastLedUpdate = millis();
    if (countingUp) {
      saturationValue++;
      setLEDs(0, HUE_SUN, saturationValue, 100);
      if (saturationValue == 100) {
        countingUp = false;
      }
    }
    else {
      saturationValue--;
      setLEDs(0, HUE_SUN, saturationValue, 100);
      if (saturationValue == 50) {
        countingUp = true;
      }
    }
    Serial.println(saturationValue);
  }
}

// Storm with lightning
void stormHandler() {
  if (millis() - lastLedUpdate >= stormInterval) {
    lastLedUpdate = millis();
    if (isLightning) {
      isLightning = false;
      setLEDs(0, HUE_STORM, 0, 100);
      stormInterval = 50;
    }
    else {
      isLightning = true;
      setLEDs(0, HUE_STORM, 100, 100);
      stormInterval = random(1000, 10000);
    }
  }
}

void setLEDs(byte channel, uint16_t h, uint16_t s, uint16_t v)
{
  if (channel < 2) {
    RGBColor color = ColorConverter.HSItoRGBW(h, s, v);
    pwm.setChannelPWM(channel * 4 + 0, map(color.red, 0, 255, 0, 4095));
    pwm.setChannelPWM(channel * 4 + 1, map(color.green, 0, 255, 0, 4095));
    pwm.setChannelPWM(channel * 4 + 2, map(color.blue, 0, 255, 0, 4095));
    pwm.setChannelPWM(channel * 4 + 3, map(color.white, 0, 255, 0, 4095));
    Serial.print("r");
    Serial.print(map(color.red, 0, 255, 0, 4095));
    Serial.print("g");
    Serial.print(map(color.green, 0, 255, 0, 4095));
    Serial.print("b");
    Serial.print(map(color.blue, 0, 255, 0, 4095));
    Serial.print("w");
    Serial.println(map(color.white, 0, 255, 0, 4095));
  }
}