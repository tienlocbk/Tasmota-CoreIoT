/*
	xsns_118_dht20.cpp - DHT20 (AHT20/AM2301B) I2C temperature and humidity sensor support for Tasmota

	This is a minimal standalone driver for DHT20-compatible sensors using the
	standard AHT20 measurement flow. Tasmota already supports AHT20 via xsns_63
	(USE_AHT2x); this module provides a dedicated DHT20 driver for setups that
	prefer an explicit DHT20 sensor entry.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_DHT20
/*********************************************************************************************\
 * DHT20 (AHT20/AM2301B) - Temperature and Humidity
 *
 * I2C Address: 0x38
 *
 * Notes:
 * - DHT20 is protocol-compatible with AHT20 (start measurement 0xAC 0x33 0x00, read 6 bytes)
 * - Parsing follows AHTx formulae:
 *     Hum[%] = 100 * raw_hum / 2^20
 *     Temp[C] = 200 * raw_temp / 2^20 - 50
\*********************************************************************************************/

#define XSNS_118              118
// Reuse I2C device enable index 95 (AHTx family) to align with existing toggles
#define XI2C_95               95  // See I2CDEVICES.md (AHT1x/2x)

#define DHT20_ADDR            0x38

// AHTx command set used by DHT20
static uint8_t DHT20_MEASURE_CMD[3] = { 0xAC, 0x33, 0x00 };
static uint8_t DHT20_CALIB_CMD[3]   = { 0xBE, 0x08, 0x00 };

#define DHT20_MEAS_DELAY_MS   80   // datasheet: ~80ms measurement duration
#define DHT20_CMD_DELAY_MS    20

struct DHT20_S {
	float   temperature = NAN;
	float   humidity = NAN;
	uint8_t valid = 0;
	uint8_t count = 0;
	char    name[6] = "DHT20";
} Dht20;

static bool Dht20WriteCmd(const uint8_t *cmd, uint8_t len) {
	TwoWire &wire = I2cGetWire();
	wire.beginTransmission(DHT20_ADDR);
	wire.write(cmd, len);
	return (0 == wire.endTransmission());
}

static uint8_t Dht20ReadStatus() {
	// AHT20 status can be read by requesting 1 byte without prior register write
	TwoWire &wire = I2cGetWire();
	wire.requestFrom((uint8_t)DHT20_ADDR, (uint8_t)1);
	if (wire.available()) {
		return wire.read();
	}
	return 0xFF; // invalid
}

static bool Dht20Init() {
	if (!I2cSetDevice(DHT20_ADDR)) { return false; }
	// Load factory calibration (AHT20/DHT20)
	if (!Dht20WriteCmd(DHT20_CALIB_CMD, 3)) { return false; }
	delay(DHT20_CMD_DELAY_MS);
	// Check calibrated bit (bit3)
	uint8_t status = Dht20ReadStatus();
	// If communication works but status unavailable, still allow progressing
	return (0xFF == status) ? true : ((status & 0x08) != 0);
}

static bool Dht20Read() {
	if (Dht20.valid) { Dht20.valid--; }

	// Trigger measurement
	if (!Dht20WriteCmd(DHT20_MEASURE_CMD, 3)) { return false; }
	delay(DHT20_MEAS_DELAY_MS);

	// Read result: 6 bytes
	uint8_t data[6] = {0};
	TwoWire &wire = I2cGetWire();
	wire.requestFrom((uint8_t)DHT20_ADDR, (uint8_t)6);
	for (uint8_t i = 0; wire.available() && i < 6; i++) {
		data[i] = wire.read();
	}
	// Busy flag bit7 of first byte
	if (data[0] & 0x80) { return false; }

	// Parse humidity (20-bit)
	uint32_t raw_h = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
	// Parse temperature (20-bit)
	uint32_t raw_t = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];

	const float denom = 1048576.0f; // 2^20
	float hum = (100.0f * (float)raw_h) / denom;
	float tem = (200.0f * (float)raw_t) / denom - 50.0f;

	Dht20.humidity    = ConvertHumidity(hum);
	Dht20.temperature = ConvertTemp(tem);

	if (isnan(Dht20.temperature) || isnan(Dht20.humidity) || (Dht20.humidity <= 0.0f)) { return false; }

	Dht20.valid = SENSOR_MAX_MISS;
	return true;
}

/********************************************************************************************/

static void Dht20Detect(void) {
	if (!Dht20Init()) { return; }
	if (Dht20Read()) {
		I2cSetActiveFound(DHT20_ADDR, Dht20.name);
		Dht20.count = 1;
	}
}

static void Dht20EverySecond(void) {
    // Poll every 2 seconds to reduce I2C load
    if (TasmotaGlobal.uptime & 1) {
        Dht20Read();
    }
}

static void Dht20Show(bool json) {
	if (Dht20.valid) {
		TempHumDewShow(json, (0 == TasmotaGlobal.tele_period), Dht20.name, Dht20.temperature, Dht20.humidity);
	}
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns118(uint32_t function)
{
	if (!I2cEnabled(XI2C_95)) { return false; }

	bool result = false;

	if (FUNC_INIT == function) {
		Dht20Detect();
	}
	else if (Dht20.count) {
		switch (function) {
			case FUNC_EVERY_SECOND:
				Dht20EverySecond();
				break;
			case FUNC_JSON_APPEND:
				Dht20Show(1);
				break;
#ifdef USE_WEBSERVER
			case FUNC_WEB_SENSOR:
				Dht20Show(0);
				break;
#endif  // USE_WEBSERVER
		}
	}
	return result;
}

#endif  // USE_DHT20
#endif  // USE_I2C

