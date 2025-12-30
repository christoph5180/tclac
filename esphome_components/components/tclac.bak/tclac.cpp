/**
 * Created by Miguel Angel Lopez on 20/07/19
 * Modified by xaxexa
 * ESPHome component refactor completed on 15.03.2024
 */
#include "esphome.h"
#include "esphome/core/defines.h"
#include "tclac.h"
#include <sstream>
#include <iomanip>


namespace esphome{
namespace tclac{

constexpr uint8_t TCL_FRAME_TYPE_STATUS = 0x04;
constexpr size_t TCL_STATUS_FRAME_MIN_SIZE = 40;

ClimateTraits tclacClimate::traits() {
	auto traits = climate::ClimateTraits();


	if (!this->supported_modes_.empty())
		traits.set_supported_modes(this->supported_modes_);
	if (!this->supported_presets_.empty())
		traits.set_supported_presets(this->supported_presets_);
	if (!this->supported_fan_modes_.empty())
		traits.set_supported_fan_modes(this->supported_fan_modes_);
	if (!this->supported_swing_modes_.empty())
		traits.set_supported_swing_modes(this->supported_swing_modes_);
	
	traits.add_supported_mode(climate::CLIMATE_MODE_OFF);			// Always expose the OFF mode.
	traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);			// Always expose the AUTO mode as well.
	traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);		// Fan AUTO mode is always available.
	traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);	// Always expose the Swing OFF mode.
	traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);// Expose the NONE preset even when presets are not configured.

	return traits;
}


void tclacClimate::setup() {

#ifdef CONF_RX_LED
	this->rx_led_pin_->setup();
	this->rx_led_pin_->digital_write(false);
#endif
#ifdef CONF_TX_LED
	this->tx_led_pin_->setup();
	this->tx_led_pin_->digital_write(false);
#endif
}

void tclacClimate::loop()  {
	if (esphome::uart::UARTDevice::available() > 0) {
		dataShow(0, true);
		size_t skipped = 0;
		bool header_found = false;
		while (esphome::uart::UARTDevice::available() > 0) {
			uint8_t byte = esphome::uart::UARTDevice::read();
			if (byte == 0xBB) {
				dataRX[0] = byte;
				header_found = true;
				break;
			}
			skipped += 1;
		}
		if (!header_found) {
			if (skipped > 0)
				ESP_LOGV("TCL", "Skipped %u byte(s) waiting for header", (unsigned) skipped);
			dataShow(0,0);
			return;
		}
		if (skipped > 0)
			ESP_LOGV("TCL", "Resynced after skipping %u byte(s)", (unsigned) skipped);
		delay(5);
		dataRX[1] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[2] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[3] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[4] = esphome::uart::UARTDevice::read();
		size_t payload_with_checksum = static_cast<size_t>(dataRX[4]) + 1;
		size_t frame_size = payload_with_checksum + 5;
		if (frame_size > sizeof(dataRX)) {
			ESP_LOGW("TCL", "Frame size %u exceeds buffer", (unsigned) frame_size);
			tclacClimate::dataShow(0,0);
			return;
		}
		esphome::uart::UARTDevice::read_array(dataRX + 5, payload_with_checksum);
		uint8_t check = getChecksum(dataRX, frame_size);
		if (check != dataRX[frame_size - 1]) {
			ESP_LOGD("TCL", "Invalid checksum %x", check);
			tclacClimate::dataShow(0,0);
			return;
		}
		tclacClimate::dataShow(0,0);
		uint8_t frame_type = dataRX[3];
		if (frame_type != TCL_FRAME_TYPE_STATUS) {
			ESP_LOGV("TCL", "Ignoring frame type 0x%02X", frame_type);
			return;
		}
		if (frame_size < TCL_STATUS_FRAME_MIN_SIZE) {
			ESP_LOGV("TCL", "Ignoring short status frame (%u bytes)", (unsigned) frame_size);
			return;
		}
		tclacClimate::readData();
	}
}

void tclacClimate::update() {
	tclacClimate::dataShow(1,1);
	this->esphome::uart::UARTDevice::write_array(poll_message_, sizeof(poll_message_));
	//auto raw = tclacClimate::getHex(poll_message_, sizeof(poll_message_));
	//ESP_LOGD("TCL", "chek status sended");
	tclacClimate::dataShow(1,0);
}

void tclacClimate::readData() {
	
	current_temperature = float((( (dataRX[17] << 8) | dataRX[18] ) / 374 - 32)/1.8);
	target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;

	//ESP_LOGD("TCL", "TEMP: %f ", current_temperature);

	bool device_is_on = (dataRX[MODE_POS] & MODE_STATUS_POWER_FLAG) != 0;
	if (device_is_on) {
		// When the AC is running, parse the state so we can show it.
		// ESP_LOGD("TCL", "AC is on");
		uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
		uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
		uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

		switch (modeswitch) {
			case MODE_AUTO:
				mode = climate::CLIMATE_MODE_AUTO;
				break;
			case MODE_COOL:
				mode = climate::CLIMATE_MODE_COOL;
				break;
			case MODE_DRY:
				mode = climate::CLIMATE_MODE_DRY;
				break;
			case MODE_FAN_ONLY:
				mode = climate::CLIMATE_MODE_FAN_ONLY;
				break;
			case MODE_HEAT:
				mode = climate::CLIMATE_MODE_HEAT;
				break;
			default:
				mode = climate::CLIMATE_MODE_AUTO;
		}

		if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
			fan_mode = climate::CLIMATE_FAN_QUIET;
		} else if (dataRX[MODE_POS] & FAN_DIFFUSE){
			fan_mode = climate::CLIMATE_FAN_DIFFUSE;
		} else {
			switch (fanspeedswitch) {
				case FAN_AUTO:
					fan_mode = climate::CLIMATE_FAN_AUTO;
					break;
				case FAN_LOW:
					fan_mode = climate::CLIMATE_FAN_LOW;
					break;
				case FAN_MIDDLE:
					fan_mode = climate::CLIMATE_FAN_MIDDLE;
					break;
				case FAN_MEDIUM:
					fan_mode = climate::CLIMATE_FAN_MEDIUM;
					break;
				case FAN_HIGH:
					fan_mode = climate::CLIMATE_FAN_HIGH;
					break;
				case FAN_FOCUS:
					fan_mode = climate::CLIMATE_FAN_FOCUS;
					break;
				default:
					fan_mode = climate::CLIMATE_FAN_AUTO;
			}
		}

		switch (swingmodeswitch) {
			case SWING_OFF: 
				swing_mode = climate::CLIMATE_SWING_OFF;
				break;
			case SWING_HORIZONTAL:
				swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
				break;
			case SWING_VERTICAL:
				swing_mode = climate::CLIMATE_SWING_VERTICAL;
				break;
			case SWING_BOTH:
				swing_mode = climate::CLIMATE_SWING_BOTH;
				break;
		}
		
		// Handle preset information received from the unit.
		preset = ClimatePreset::CLIMATE_PRESET_NONE;
		if (dataRX[7] & (1 << 6)){
			preset = ClimatePreset::CLIMATE_PRESET_ECO;
		} else if (dataRX[9] & (1 << 2)){
			preset = ClimatePreset::CLIMATE_PRESET_COMFORT;
		} else if (dataRX[19] & (1 << 0)){
			preset = ClimatePreset::CLIMATE_PRESET_SLEEP;
		}
		
	} else {
		// If the AC is off, pretend every mode is OFF.
		mode = climate::CLIMATE_MODE_OFF;
		//fan_mode = climate::CLIMATE_FAN_OFF;
		swing_mode = climate::CLIMATE_SWING_OFF;
		preset = ClimatePreset::CLIMATE_PRESET_NONE;
	}
	// Publish the freshly parsed climate state.
	this->publish_state();
	allow_take_control = true;
   }

// Climate control
void tclacClimate::control(const ClimateCall &call) {
	// Figure out which climate mode should be sent (call override or current).
	if (call.get_mode().has_value()){
		switch_climate_mode = call.get_mode().value();
		ESP_LOGD("TCL", "Get MODE from call");
	} else {
		switch_climate_mode = mode;
		ESP_LOGD("TCL", "Get MODE from AC");
	}
	
	// Figure out which preset should be sent.
	if (call.get_preset().has_value()){
		switch_preset = call.get_preset().value();
	} else {
		switch_preset = preset.value();
	}
	
	// Figure out which fan mode should be sent.
	if (call.get_fan_mode().has_value()){
		switch_fan_mode = call.get_fan_mode().value();
	} else {
		switch_fan_mode = fan_mode.value();
	}
	
	// Figure out which swing mode should be sent.
	if (call.get_swing_mode().has_value()){
		switch_swing_mode = call.get_swing_mode().value();
	} else {
		// If nothing was provided, reuse the last known value so behavior stays unchanged.
		switch_swing_mode = swing_mode;
	}
	
	// Encode the requested target temperature (AC expects 31 - value).
	if (call.get_target_temperature().has_value()) {
		target_temperature_set = 31-(int)call.get_target_temperature().value();
	} else {
		target_temperature_set = 31-(int)target_temperature;
	}
	
	is_call_control = true;
	takeControl();
	allow_take_control = true;
}
	
	
void tclacClimate::takeControl() {
	
	dataTX[7]  = 0b00000000;
	dataTX[8]  = 0b00000000;
	dataTX[9]  = 0b00000000;
	dataTX[10] = 0b00000000;
	dataTX[11] = 0b00000000;
	dataTX[19] = 0b00000000;
	dataTX[32] = 0b00000000;
	dataTX[33] = 0b00000000;
	
	if (is_call_control != true){
		ESP_LOGD("TCL", "Get MODE from AC for force config");
		switch_climate_mode = mode;
		switch_preset = preset.value();
		switch_fan_mode = fan_mode.value();
		switch_swing_mode = swing_mode;
		target_temperature_set = 31-(int)target_temperature;
	}
	
	// Toggle the beeper according to the configuration flag.

	if (beeper_status_){
		ESP_LOGD("TCL", "Beep mode ON");
		dataTX[7] += 0b00100000;
	} else {
		ESP_LOGD("TCL", "Beep mode OFF");
		dataTX[7] += 0b00000000;
	}
	
	// Toggle the AC display according to the configuration flag.

	// Only enable the display while the AC runs in an active mode.
	
	// WARNING: turning the display off forces the AC into AUTO mode!
	
	if ((display_status_) && (switch_climate_mode != climate::CLIMATE_MODE_OFF)){
		ESP_LOGD("TCL", "Dispaly turn ON");
		dataTX[7] += 0b01000000;
	} else {
		ESP_LOGD("TCL", "Dispaly turn OFF");
		dataTX[7] += 0b00000000;
	}
		
	// Encode the selected climate mode into the control frame.
	switch (switch_climate_mode) {
		case climate::CLIMATE_MODE_OFF:
			dataTX[7] += 0b00000000;
			dataTX[8] += 0b00000000;
			break;
		case climate::CLIMATE_MODE_AUTO:
			dataTX[7] += MODE_COMMAND_POWER_FLAG;
			dataTX[8] += 0b00001000;
			break;
		case climate::CLIMATE_MODE_COOL:
			dataTX[7] += MODE_COMMAND_POWER_FLAG;
			dataTX[8] += 0b00000011;	
			break;
		case climate::CLIMATE_MODE_DRY:
			dataTX[7] += MODE_COMMAND_POWER_FLAG;
			dataTX[8] += 0b00000010;	
			break;
		case climate::CLIMATE_MODE_FAN_ONLY:
			dataTX[7] += MODE_COMMAND_POWER_FLAG;
			dataTX[8] += 0b00000111;	
			break;
		case climate::CLIMATE_MODE_HEAT:
			dataTX[7] += MODE_COMMAND_POWER_FLAG;
			dataTX[8] += 0b00000001;	
			break;
	}

	// Encode the selected fan mode into the control frame.
	switch(switch_fan_mode) {
		case climate::CLIMATE_FAN_AUTO:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000000;
			break;
		case climate::CLIMATE_FAN_QUIET:
			dataTX[8]	+= 0b10000000;
			dataTX[10]	+= 0b00000000;
			break;
		case climate::CLIMATE_FAN_LOW:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000001;
			break;
		case climate::CLIMATE_FAN_MIDDLE:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000110;
			break;
		case climate::CLIMATE_FAN_MEDIUM:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000011;
			break;
		case climate::CLIMATE_FAN_HIGH:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000111;
			break;
		case climate::CLIMATE_FAN_FOCUS:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000101;
			break;
		case climate::CLIMATE_FAN_DIFFUSE:
			dataTX[8]	+= 0b01000000;
			dataTX[10]	+= 0b00000000;
			break;
	}
	
	// Encode the requested swing mode bits.
	switch(switch_swing_mode) {
		case climate::CLIMATE_SWING_OFF:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_VERTICAL:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_HORIZONTAL:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00001000;
			break;
		case climate::CLIMATE_SWING_BOTH:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00001000;  
			break;
	}
	
	// Encode the requested preset bits.
	switch(switch_preset) {
		case ClimatePreset::CLIMATE_PRESET_NONE:
			break;
		case ClimatePreset::CLIMATE_PRESET_ECO:
			dataTX[7]	+= 0b10000000;
			break;
		case ClimatePreset::CLIMATE_PRESET_SLEEP:
			dataTX[19]	+= 0b00000001;
			break;
		case ClimatePreset::CLIMATE_PRESET_COMFORT:
			dataTX[8]	+= 0b00010000;
			break;
	}

        // Louver control helper information.
		// Vertical louver handling.
		// Vertical swing bits [byte 10, mask 0b00111000]:
		// 000 - swing off, louver stays in its last/fixed position.
		// 111 - swing enabled per the requested pattern.
		// Vertical swing macro mode (fixation ignored when swing runs) [byte 32, mask 0b00011000]:
		// 01 - sweep from top to bottom (default).
		// 10 - sweep only in the upper half.
		// 11 - sweep only in the lower half.
		// Vertical fixation position when swing is disabled [byte 32, mask 0b00000111]:
		// 000 - no fixation (default).
		// 001 - fix at the top.
		// 010 - fix between top and center.
		// 011 - fix at the center.
		// 100 - fix between center and bottom.
		// 101 - fix at the bottom.
		// Horizontal louver handling.
		// Horizontal swing bit [byte 11, mask 0b00001000]:
		// 0 - swing off, louvers stay where they were.
		// 1 - swing enabled.
		// Horizontal swing macro mode (fixation ignored while swinging) [byte 33, mask 0b00111000]:
		// 001 - sweep from left to right (default).
		// 010 - sweep primarily on the left side.
		// 011 - sweep around the center.
		// 100 - sweep primarily on the right side.
		// Horizontal fixation position when swing is disabled [byte 33, mask 0b00000111]:
		// 000 - no fixation (default).
		// 001 - fix to the far left.
		// 010 - fix between the left edge and center.
		// 011 - fix at the center.
		// 100 - fix between the center and right edge.
		// 101 - fix to the far right.
		
		
	// Apply the requested vertical swing direction.
	switch(vertical_swing_direction_) {
		case VerticalSwingDirection::UP_DOWN:
			dataTX[32]	+= 0b00001000;
			ESP_LOGD("TCL", "Vertical swing: up-down");
			break;
		case VerticalSwingDirection::UPSIDE:
			dataTX[32]	+= 0b00010000;
			ESP_LOGD("TCL", "Vertical swing: upper");
			break;
		case VerticalSwingDirection::DOWNSIDE:
			dataTX[32]	+= 0b00011000;
			ESP_LOGD("TCL", "Vertical swing: downer");
			break;
	}
	// Apply the requested horizontal swing direction.
	switch(horizontal_swing_direction_) {
		case HorizontalSwingDirection::LEFT_RIGHT:
			dataTX[33]	+= 0b00001000;
			ESP_LOGD("TCL", "Horizontal swing: left-right");
			break;
		case HorizontalSwingDirection::LEFTSIDE:
			dataTX[33]	+= 0b00010000;
			ESP_LOGD("TCL", "Horizontal swing: lefter");
			break;
		case HorizontalSwingDirection::CENTER:
			dataTX[33]	+= 0b00011000;
			ESP_LOGD("TCL", "Horizontal swing: center");
			break;
		case HorizontalSwingDirection::RIGHTSIDE:
			dataTX[33]	+= 0b00100000;
			ESP_LOGD("TCL", "Horizontal swing: righter");
			break;
	}
	// Apply the requested vertical fixation position.
	switch(vertical_direction_) {
		case AirflowVerticalDirection::LAST:
			dataTX[32]	+= 0b00000000;
			ESP_LOGD("TCL", "Vertical fix: last position");
			break;
		case AirflowVerticalDirection::MAX_UP:
			dataTX[32]	+= 0b00000001;
			ESP_LOGD("TCL", "Vertical fix: up");
			break;
		case AirflowVerticalDirection::UP:
			dataTX[32]	+= 0b00000010;
			ESP_LOGD("TCL", "Vertical fix: upper");
			break;
		case AirflowVerticalDirection::CENTER:
			dataTX[32]	+= 0b00000011;
			ESP_LOGD("TCL", "Vertical fix: center");
			break;
		case AirflowVerticalDirection::DOWN:
			dataTX[32]	+= 0b00000100;
			ESP_LOGD("TCL", "Vertical fix: downer");
			break;
		case AirflowVerticalDirection::MAX_DOWN:
			dataTX[32]	+= 0b00000101;
			ESP_LOGD("TCL", "Vertical fix: down");
			break;
	}
	// Apply the requested horizontal fixation position.
	switch(horizontal_direction_) {
		case AirflowHorizontalDirection::LAST:
			dataTX[33]	+= 0b00000000;
			ESP_LOGD("TCL", "Horizontal fix: last position");
			break;
		case AirflowHorizontalDirection::MAX_LEFT:
			dataTX[33]	+= 0b00000001;
			ESP_LOGD("TCL", "Horizontal fix: left");
			break;
		case AirflowHorizontalDirection::LEFT:
			dataTX[33]	+= 0b00000010;
			ESP_LOGD("TCL", "Horizontal fix: lefter");
			break;
		case AirflowHorizontalDirection::CENTER:
			dataTX[33]	+= 0b00000011;
			ESP_LOGD("TCL", "Horizontal fix: center");
			break;
		case AirflowHorizontalDirection::RIGHT:
			dataTX[33]	+= 0b00000100;
			ESP_LOGD("TCL", "Horizontal fix: righter");
			break;
		case AirflowHorizontalDirection::MAX_RIGHT:
			dataTX[33]	+= 0b00000101;
			ESP_LOGD("TCL", "Horizontal fix: right");
			break;
	}

	// Set the encoded temperature byte.
	dataTX[9] = target_temperature_set;
		
	// Assemble the outbound control frame.
	dataTX[0] = 0xBB;	// Frame header byte.
	dataTX[1] = 0x00;	// Frame header byte.
	dataTX[2] = 0x01;	// Frame header byte.
	dataTX[3] = 0x03;	// 0x03 = control frame, 0x04 = status frame.
	dataTX[4] = 0x20;	// 0x20 is the control payload length, 0x19 is the status payload length.
	dataTX[5] = 0x03;	//??
	dataTX[6] = 0x01;	//??
	//dataTX[7] = 0x64;	//eco,display,beep,ontimerenable, offtimerenable,power,0,0
	//dataTX[8] = 0x08;	//mute,0,turbo,health, mode(4) mode 01 heat, 02 dry, 03 cool, 07 fan, 08 auto, health(+16), 41=turbo-heat 43=turbo-cool (turbo = 0x40+ 0x01..0x08)
	//dataTX[9] = 0x0f;	//0 -31 ;    15 - 16 0,0,0,0, temp(4) settemp 31 - x
	//dataTX[10] = 0x00;	//0,timerindicator,swingv(3),fan(3) fan+swing modes //0=auto 1=low 2=med 3=high
	//dataTX[11] = 0x00;	//0,offtimer(6),0
	dataTX[12] = 0x00;	//fahrenheit,ontimer(6),0 cf 80=f 0=c
	dataTX[13] = 0x01;	//??
	dataTX[14] = 0x00;	//0,0,halfdegree,0,0,0,0,0
	dataTX[15] = 0x00;	//??
	dataTX[16] = 0x00;	//??
	dataTX[17] = 0x00;	//??
	dataTX[18] = 0x00;	//??
	//dataTX[19] = 0x00;	//sleep on = 1 off=0
	dataTX[20] = 0x00;	//??
	dataTX[21] = 0x00;	//??
	dataTX[22] = 0x00;	//??
	dataTX[23] = 0x00;	//??
	dataTX[24] = 0x00;	//??
	dataTX[25] = 0x00;	//??
	dataTX[26] = 0x00;	//??
	dataTX[27] = 0x00;	//??
	dataTX[28] = 0x00;	//??
	dataTX[30] = 0x00;	//??
	dataTX[31] = 0x00;	//??
	// 0,0,0, vertical swing bits (2), vertical fixation bits (3).
	// 0,0, horizontal swing bits (3), horizontal fixation bits (3).
	dataTX[34] = 0x00;	//??
	dataTX[35] = 0x00;	//??
	dataTX[36] = 0x00;	//??
	dataTX[37] = 0xFF;	// Checksum byte.
	dataTX[37] = tclacClimate::getChecksum(dataTX, sizeof(dataTX));

	tclacClimate::sendData(dataTX, sizeof(dataTX));
	allow_take_control = false;
	is_call_control = false;
}

// Send the prepared frame to the AC.
void tclacClimate::sendData(uint8_t * message, uint8_t size) {
	tclacClimate::dataShow(1,1);
	//Serial.write(message, size);
	this->esphome::uart::UARTDevice::write_array(message, size);
	//auto raw = getHex(message, size);
	ESP_LOGD("TCL", "Message to TCL sended...");
	tclacClimate::dataShow(1,0);
}

// Convert a byte array into a readable hex string.
std::string tclacClimate::getHex(const byte *message, size_t size) {
	std::ostringstream oss;
	for (size_t i = 0; i < size; ++i) {
		oss << std::hex
			<< std::uppercase
			<< std::setw(2)
			<< std::setfill('0')
			<< static_cast<int>(message[i]);
		if (i + 1 < size)
			oss << ' ';
	}
	std::string s = oss.str();
	std::transform(s.begin(), s.end(), s.begin(), ::toupper);
	return s;
}

// Calculate the XOR checksum for a frame.
uint8_t tclacClimate::getChecksum(const byte * message, size_t size) {
	uint8_t position = size - 1;
	uint8_t crc = 0;
	for (int i = 0; i < position; i++)
		crc ^= message[i];
	return crc;
}

// Blink LEDs to indicate RX/TX activity.
void tclacClimate::dataShow(bool flow, bool shine) {
	if (module_display_status_){
		if (flow == 0){
			if (shine == 1){
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(false);
#endif
			}
		}
		if (flow == 1) {
			if (shine == 1){
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(false);
#endif
			}
		}
	}
}

// Helpers for manipulating configuration-backed state.

// Update the stored beeper state.
void tclacClimate::set_beeper_state(bool state) {
	this->beeper_status_ = state;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Update the stored AC display state.
void tclacClimate::set_display_state(bool state) {
	this->display_status_ = state;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Update whether forced control is currently enabled.
void tclacClimate::set_force_mode_state(bool state) {
	this->force_mode_status_ = state;
}
// Assign the RX LED pin.

#ifdef CONF_RX_LED
void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) {
	this->rx_led_pin_ = rx_led_pin;
}
#endif
// Assign the TX LED pin.

#ifdef CONF_TX_LED
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) {
	this->tx_led_pin_ = tx_led_pin;
}
#endif
// Update the module display flag.
void tclacClimate::set_module_display_state(bool state) {
	this->module_display_status_ = state;
}
// Update the stored vertical airflow fixation target.
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) {
	this->vertical_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Update the stored horizontal airflow fixation target.
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection direction) {
	this->horizontal_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Update the stored vertical swing direction.
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) {
	this->vertical_swing_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Register supported climate modes.
void tclacClimate::set_supported_modes(const std::set<climate::ClimateMode> &modes) {
	this->supported_modes_ = modes;
}
// Update the stored horizontal swing direction.
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection direction) {
	horizontal_swing_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
// Register supported fan modes.
void tclacClimate::set_supported_fan_modes(const std::set<climate::ClimateFanMode> &modes){
	this->supported_fan_modes_ = modes;
}
// Register supported swing modes.
void tclacClimate::set_supported_swing_modes(const std::set<climate::ClimateSwingMode> &modes) {
	this->supported_swing_modes_ = modes;
}
// Register supported presets.
void tclacClimate::set_supported_presets(const std::set<climate::ClimatePreset> &presets) {
  this->supported_presets_ = presets;
}

}
}
