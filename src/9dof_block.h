#ifndef _9DOF_BLOCK_H__
#define _9DOF_BLOCK_H__

#include <mraa.hpp>

namespace reg {

const uint8_t WHO_AM_I_G = 0x0F;
const uint8_t CTRL_REG1_G = 0x20;
const uint8_t CTRL_REG2_G = 0x21;
const uint8_t CTRL_REG3_G = 0x22;
const uint8_t CTRL_REG4_G = 0x23;
const uint8_t CTRL_REG5_G = 0x24;
const uint8_t REFERENCE_G = 0x25;
const uint8_t STATUS_REG_G = 0x27;
const uint8_t OUT_X_L_G = 0x28;
const uint8_t OUT_X_H_G = 0x29;
const uint8_t OUT_Y_L_G = 0x2A;
const uint8_t OUT_Y_H_G = 0x2B;
const uint8_t OUT_Z_L_G = 0x2C;
const uint8_t OUT_Z_H_G = 0x2D;
const uint8_t FIFO_CTRL_REG_G = 0x2E;
const uint8_t FIFO_SRC_REG_G = 0x2F;
const uint8_t INT1_CFG_G = 0x30;
const uint8_t INT1_SRC_G = 0x31;
const uint8_t INT1_THS_XH_G = 0x32;
const uint8_t INT1_THS_XL_G = 0x33;
const uint8_t INT1_THS_YH_G = 0x34;
const uint8_t INT1_THS_YL_G = 0x35;
const uint8_t INT1_THS_ZH_G = 0x36;
const uint8_t INT1_THS_ZL_G = 0x37;
const uint8_t INT1_DURATION_G = 0x38;

const uint8_t OUT_TEMP_L_XM = 0x05;
const uint8_t OUT_TEMP_H_XM = 0x06;
const uint8_t STATUS_REG_M = 0x07;
const uint8_t OUT_X_L_M = 0x08;
const uint8_t OUT_X_H_M = 0x09;
const uint8_t OUT_Y_L_M = 0x0A;
const uint8_t OUT_Y_H_M = 0x0B;
const uint8_t OUT_Z_L_M = 0x0C;
const uint8_t OUT_Z_H_M = 0x0D;
const uint8_t WHO_AM_I_XM =0x0F;
const uint8_t INT_CTRL_REG_M = 0x12;
const uint8_t INT_SRC_REG_M = 0x13;
const uint8_t INT_THS_L_M = 0x14;
const uint8_t INT_THS_H_M = 0x15;
const uint8_t OFFSET_X_L_M = 0x16;
const uint8_t OFFSET_X_H_M = 0x17;
const uint8_t OFFSET_Y_L_M = 0x18;
const uint8_t OFFSET_Y_H_M = 0x19;
const uint8_t OFFSET_Z_L_M = 0x1A;
const uint8_t OFFSET_Z_H_M = 0x1B;
const uint8_t REFERENCE_X	= 0x1C;
const uint8_t REFERENCE_Y = 0x1D;
const uint8_t REFERENCE_Z = 0x1E;
const uint8_t CTRL_REG0_XM = 0x1F;
const uint8_t CTRL_REG1_XM = 0x20;
const uint8_t CTRL_REG2_XM = 0x21;
const uint8_t CTRL_REG3_XM = 0x22;
const uint8_t CTRL_REG4_XM = 0x23;
const uint8_t CTRL_REG5_XM = 0x24;
const uint8_t CTRL_REG6_XM = 0x25;
const uint8_t CTRL_REG7_XM = 0x26;
const uint8_t STATUS_REG_A = 0x27;
const uint8_t OUT_X_L_A = 0x28;
const uint8_t OUT_X_H_A = 0x29;
const uint8_t OUT_Y_L_A = 0x2A;
const uint8_t OUT_Y_H_A = 0x2B;
const uint8_t OUT_Z_L_A = 0x2C;
const uint8_t OUT_Z_H_A = 0x2D;
const uint8_t FIFO_CTRL_REG = 0x2E;
const uint8_t FIFO_SRC_REG = 0x2F;
const uint8_t INT_GEN_1_REG = 0x30;
const uint8_t INT_GEN_1_SRC = 0x31;
const uint8_t INT_GEN_1_THS = 0x32;
const uint8_t INT_GEN_1_DURATION = 0x33;
const uint8_t INT_GEN_2_REG = 0x34;
const uint8_t INT_GEN_2_SRC = 0x35;
const uint8_t INT_GEN_2_THS = 0x36;
const uint8_t INT_GEN_2_DURATION = 0x37;
const uint8_t CLICK_CFG = 0x38;
const uint8_t CLICK_SRC = 0x39;
const uint8_t CLICK_THS = 0x3A;
const uint8_t TIME_LIMIT = 0x3B;
const uint8_t TIME_LATENCY = 0x3C;
const uint8_t TIME_WINDOW = 0x3D;
const uint8_t ACT_THS = 0x3E;
const uint8_t ACT_DUR = 0x3F;

};

namespace bit {
	uint8_t TEMP_EN = 0x80;
	uint8_t ADDR_AUTO_INC = 0x80;
	uint8_t MAG_RATE = 0x1C;
	uint8_t MAG_RES = 0x60;

	uint8_t GYRO_EN = 0x08;
	uint8_t GYRO_DR = 0xC0;
	uint8_t GYRO_BW = 0x30;
};

struct sensor_t {
	union {
		struct {
		int16_t x;
		int16_t y;
		int16_t z;
		};
		uint8_t raw_bytes[6];
	};
};


class IMUBlock {
private:
	mraa::I2c* i2c_;
	uint8_t xm_address_;
	uint8_t g_address_;

	void write_byte(uint8_t reg, uint8_t byte) {
		uint8_t buf[2];
		buf[0] = reg;
		buf[1] = byte;

		i2c_->write(buf, 2);
	}

	uint8_t read_byte(uint8_t reg) {
		uint8_t val = i2c_->readReg(reg);
		return val;
	}

	int16_t read_val(uint8_t reg) {
		int16_t val;
		i2c_->readBytesReg(reg | bit::ADDR_AUTO_INC, (uint8_t*)&val, 2);
		//printf("reg: 0x%x val: 0x%x\n", reg, val);
		return val;
	}

	sensor_t read_sensor(uint8_t reg) {
		sensor_t val;
		i2c_->readBytesReg(reg | bit::ADDR_AUTO_INC, val.raw_bytes, 6);
		return val;
	}

	void use_xm() {
		i2c_->address(xm_address_);
	}

	void use_g() {
		i2c_->address(g_address_);
	}

public:

	static const int EdisonBus = 1;

	IMUBlock(uint8_t xm_address=0x1D, uint8_t g_address=0x6B) {
		i2c_ = new mraa::I2c(EdisonBus);
		xm_address_ = xm_address;
		g_address_ = g_address;

		setGyroEnable(true);
	}

	~IMUBlock() {
		delete i2c_;
	}

	void setup() {

	}

	void setAccelRate(uint8_t rate) {
		use_xm();
		uint8_t reg1 = read_byte(reg::CTRL_REG1_XM);
		reg1 = (reg1 & 0x0F) | (rate << 4);
		write_byte(reg::CTRL_REG1_XM, reg1);
	}

	void setMagMode(uint8_t mode) {
		use_xm();
		uint8_t reg7 = read_byte(reg::CTRL_REG7_XM);
		reg7 = (reg7 & ~(0x03)) | (mode & 0x03);
		write_byte(reg::CTRL_REG7_XM, reg7);
	}

	void setMagRate(uint8_t rate) {
		use_xm();
		uint8_t reg5 = read_byte(reg::CTRL_REG5_XM);
		reg5 = (reg5 & ~bit::MAG_RATE) | ((rate & 0x07) << 2);
		write_byte(reg::CTRL_REG5_XM, reg5);
	}

	void setMagRes(uint8_t res) {
		use_xm();
		uint8_t reg5 = read_byte(reg::CTRL_REG5_XM);
		reg5 = (reg5 & ~bit::MAG_RES) | ((res & 0x03) << 5);
		write_byte(reg::CTRL_REG5_XM, reg5);
	}

	void setGyroEnable(bool en) {
		use_g();
		uint8_t reg1 = read_byte(reg::CTRL_REG1_G);
		if (en) {
			reg1 = reg1 | bit::GYRO_EN;
		} else {
			reg1 = reg1 & bit::GYRO_EN;
		}
		write_byte(reg::CTRL_REG1_G, reg1);
	}

	void setGyroRate(uint8_t rate) {
		use_g();
		uint8_t reg1 = read_byte(reg::CTRL_REG1_G);
		reg1 = (reg1 & ~bit::GYRO_DR) | ((rate << 6) & 0x03);

		write_byte(reg::CTRL_REG1_G, reg1);
	}

	void setGyroBandwidth(uint8_t bw) {
		use_g();
		uint8_t reg1 = read_byte(reg::CTRL_REG1_G);
		reg1 = (reg1 & ~bit::GYRO_BW) | ((bw << 4) & 0x03);

		write_byte(reg::CTRL_REG1_G, reg1);
	}

	void setTempEnable(bool temp) {
		use_xm();
		uint8_t reg5 = read_byte(reg::CTRL_REG5_XM);
		if (temp) {
			reg5 = reg5 | bit::TEMP_EN;
		} else {
			reg5 = reg5 & bit::TEMP_EN;
		}

		write_byte(reg::CTRL_REG5_XM, reg5);
		read_byte(reg::CTRL_REG5_XM);
	}

	int16_t readTemp() {
		use_xm();
		return read_val(reg::OUT_TEMP_L_XM);
	}


	sensor_t readAccel() {
		use_xm();
		return read_sensor(reg::OUT_X_L_A);
	}

	sensor_t readGyro() {
		use_g();
		return read_sensor(reg::OUT_X_L_G);
	}

	sensor_t readMag() {
		use_xm();
		return read_sensor(reg::OUT_X_L_M);
	}




};






#endif
