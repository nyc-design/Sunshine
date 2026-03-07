<script setup>
import { ref } from 'vue'
import PlatformLayout from '../../PlatformLayout.vue'
import Checkbox from "../../Checkbox.vue";

const props = defineProps([
  'platform',
  'config'
])

const config = ref(props.config)
</script>

<template>
  <div id="input" class="config-page">
    <!-- Enable Gamepad Input -->
    <Checkbox class="mb-3"
              id="controller"
              locale-prefix="config"
              v-model="config.controller"
              default="true"
    ></Checkbox>

    <!-- Emulated Gamepad Type -->
    <div class="mb-3" v-if="config.controller === 'enabled' && platform !== 'macos'">
      <label for="gamepad" class="form-label">{{ $t('config.gamepad') }}</label>
      <select id="gamepad" class="form-select" v-model="config.gamepad">
        <option value="auto">{{ $t('_common.auto') }}</option>

        <PlatformLayout :platform="platform">
          <template #freebsd>
            <option value="switch">{{ $t("config.gamepad_switch") }}</option>
            <option value="xone">{{ $t("config.gamepad_xone") }}</option>
          </template>

          <template #linux>
            <option value="ds5">{{ $t("config.gamepad_ds5") }}</option>
            <option value="switch">{{ $t("config.gamepad_switch") }}</option>
            <option value="xone">{{ $t("config.gamepad_xone") }}</option>
          </template>

          <template #windows>
            <option value="ds4">{{ $t('config.gamepad_ds4') }}</option>
            <option value="x360">{{ $t('config.gamepad_x360') }}</option>
          </template>
        </PlatformLayout>
      </select>
      <div class="form-text">{{ $t('config.gamepad_desc') }}</div>
    </div>

    <!-- Controller transport -->
    <div class="mb-3" v-if="config.controller === 'enabled' && (platform === 'linux' || platform === 'windows')">
      <label for="controller_transport" class="form-label">{{ $t('config.controller_transport') }}</label>
      <select id="controller_transport" class="form-select" v-model="config.controller_transport">
        <option value="host">{{ $t('config.controller_transport_host') }}</option>
        <option value="esp32">{{ $t('config.controller_transport_esp32') }}</option>
      </select>
      <div class="form-text">{{ $t('config.controller_transport_desc') }}</div>
    </div>

    <!-- ESP32 serial transport options -->
    <template v-if="config.controller === 'enabled' && (platform === 'linux' || platform === 'windows') && config.controller_transport === 'esp32'">
      <div class="mb-3">
        <label for="esp32_serial_port" class="form-label">{{ $t('config.esp32_serial_port') }}</label>
        <input id="esp32_serial_port"
               type="text"
               class="form-control"
               placeholder="COM3 or /dev/ttyACM0"
               v-model="config.esp32_serial_port" />
        <div class="form-text">{{ $t('config.esp32_serial_port_desc') }}</div>
      </div>

      <div class="mb-3">
        <label for="esp32_baud" class="form-label">{{ $t('config.esp32_baud') }}</label>
        <input id="esp32_baud"
               type="number"
               class="form-control"
               placeholder="115200"
               v-model="config.esp32_baud" />
        <div class="form-text">{{ $t('config.esp32_baud_desc') }}</div>
      </div>

      <div class="mb-3">
        <label for="esp32_mode" class="form-label">{{ $t('config.esp32_mode') }}</label>
        <select id="esp32_mode" class="form-select" v-model="config.esp32_mode">
          <option value="gamepad">{{ $t('config.esp32_mode_gamepad') }}</option>
          <option value="switch_controller">{{ $t('config.esp32_mode_switch_controller') }}</option>
          <option value="wired_switch_pro_controller">{{ $t('config.esp32_mode_wired_switch_pro_controller') }}</option>
          <option value="switch_wired">{{ $t('config.esp32_mode_switch_wired') }}</option>
        </select>
        <div class="form-text">{{ $t('config.esp32_mode_desc') }}</div>
      </div>

      <div class="mb-3">
        <label for="esp32_delivery_policy" class="form-label">{{ $t('config.esp32_delivery_policy') }}</label>
        <select id="esp32_delivery_policy" class="form-select" v-model="config.esp32_delivery_policy">
          <option value="auto">{{ $t('config.esp32_delivery_policy_auto') }}</option>
          <option value="wired">{{ $t('config.esp32_delivery_policy_wired') }}</option>
          <option value="bluetooth">{{ $t('config.esp32_delivery_policy_bluetooth') }}</option>
          <option value="websocket">{{ $t('config.esp32_delivery_policy_websocket') }}</option>
          <option value="http">{{ $t('config.esp32_delivery_policy_http') }}</option>
        </select>
        <div class="form-text">{{ $t('config.esp32_delivery_policy_desc') }}</div>
      </div>
    </template>

    <!-- Additional options based on gamepad type -->
    <template v-if="config.controller === 'enabled'">
      <template v-if="config.gamepad === 'ds4' || config.gamepad === 'ds5' || (config.gamepad === 'auto' && platform !== 'macos')">
        <div class="mb-3 accordion">
          <div class="accordion-item">
            <h2 class="accordion-header">
              <button class="accordion-button" type="button" data-bs-toggle="collapse"
                      data-bs-target="#panelsStayOpen-collapseOne">
                {{ $t(config.gamepad === 'ds4' ? 'config.gamepad_ds4_manual' : (config.gamepad === 'ds5' ? 'config.gamepad_ds5_manual' : 'config.gamepad_auto')) }}
              </button>
            </h2>
            <div id="panelsStayOpen-collapseOne" class="accordion-collapse collapse show"
                 aria-labelledby="panelsStayOpen-headingOne">
              <div class="accordion-body">
                <!-- Automatic detection options (for Windows and Linux) -->
                <template v-if="config.gamepad === 'auto' && (platform === 'windows' || platform === 'linux')">
                  <!-- Gamepad with motion-capability as DS4(Windows)/DS5(Linux) -->
                  <Checkbox class="mb-3"
                            id="motion_as_ds4"
                            locale-prefix="config"
                            v-model="config.motion_as_ds4"
                            default="true"
                  ></Checkbox>
                  <!-- Gamepad with touch-capability as DS4(Windows)/DS5(Linux) -->
                  <Checkbox class="mb-3"
                            id="touchpad_as_ds4"
                            locale-prefix="config"
                            v-model="config.touchpad_as_ds4"
                            default="true"
                  ></Checkbox>
                </template>
                <!-- DS4 option: DS4 back button as touchpad click (on Automatic: Windows only) -->
                <template v-if="config.gamepad === 'ds4' || (config.gamepad === 'auto' && platform === 'windows')">
                  <Checkbox class="mb-3"
                            id="ds4_back_as_touchpad_click"
                            locale-prefix="config"
                            v-model="config.ds4_back_as_touchpad_click"
                            default="true"
                  ></Checkbox>
                </template>
                <!-- DS5 Option: Controller MAC randomization (on Automatic: Linux only) -->
                <template v-if="config.gamepad === 'ds5' || (config.gamepad === 'auto' && platform === 'linux')">
                  <Checkbox class="mb-3"
                            id="ds5_inputtino_randomize_mac"
                            locale-prefix="config"
                            v-model="config.ds5_inputtino_randomize_mac"
                            default="true"
                  ></Checkbox>
                </template>
              </div>
            </div>
          </div>
        </div>
      </template>
    </template>

    <!-- Home/Guide Button Emulation Timeout -->
    <div class="mb-3" v-if="config.controller === 'enabled'">
      <label for="back_button_timeout" class="form-label">{{ $t('config.back_button_timeout') }}</label>
      <input type="text" class="form-control" id="back_button_timeout" placeholder="-1"
             v-model="config.back_button_timeout" />
      <div class="form-text">{{ $t('config.back_button_timeout_desc') }}</div>
    </div>

    <!-- Enable Keyboard Input -->
    <hr>
    <Checkbox class="mb-3"
              id="keyboard"
              locale-prefix="config"
              v-model="config.keyboard"
              default="true"
    ></Checkbox>

    <!-- Key Repeat Delay-->
    <div class="mb-3" v-if="config.keyboard === 'enabled' && platform === 'windows'">
      <label for="key_repeat_delay" class="form-label">{{ $t('config.key_repeat_delay') }}</label>
      <input type="text" class="form-control" id="key_repeat_delay" placeholder="500"
             v-model="config.key_repeat_delay" />
      <div class="form-text">{{ $t('config.key_repeat_delay_desc') }}</div>
    </div>

    <!-- Key Repeat Frequency-->
    <div class="mb-3" v-if="config.keyboard === 'enabled' && platform === 'windows'">
      <label for="key_repeat_frequency" class="form-label">{{ $t('config.key_repeat_frequency') }}</label>
      <input type="text" class="form-control" id="key_repeat_frequency" placeholder="24.9"
             v-model="config.key_repeat_frequency" />
      <div class="form-text">{{ $t('config.key_repeat_frequency_desc') }}</div>
    </div>

    <!-- Always send scancodes -->
    <Checkbox v-if="config.keyboard === 'enabled' && platform === 'windows'"
              class="mb-3"
              id="always_send_scancodes"
              locale-prefix="config"
              v-model="config.always_send_scancodes"
              default="true"
    ></Checkbox>

    <!-- Mapping Key AltRight to Key Windows -->
    <Checkbox v-if="config.keyboard === 'enabled'"
              class="mb-3"
              id="key_rightalt_to_key_win"
              locale-prefix="config"
              v-model="config.key_rightalt_to_key_win"
              default="false"
    ></Checkbox>

    <!-- Enable Mouse Input -->
    <hr>
    <Checkbox class="mb-3"
              id="mouse"
              locale-prefix="config"
              v-model="config.mouse"
              default="true"
    ></Checkbox>

    <!-- High resolution scrolling support -->
    <Checkbox v-if="config.mouse === 'enabled'"
              class="mb-3"
              id="high_resolution_scrolling"
              locale-prefix="config"
              v-model="config.high_resolution_scrolling"
              default="true"
    ></Checkbox>

    <!-- Native pen/touch support -->
    <Checkbox v-if="config.mouse === 'enabled'"
              class="mb-3"
              id="native_pen_touch"
              locale-prefix="config"
              v-model="config.native_pen_touch"
              default="true"
    ></Checkbox>
  </div>
</template>

<style scoped>

</style>
