/**
 * atc3dg.hpp
 * 
 * Contains module definitions to interface with Ascension 3D Guidance
 * trakSTAR-units.
 * 
 * All of this code is highly experimental. Some tracker commands were
 * extracted from USB sniffing records, their exact functionality is thus
 * unknown.
 * There is no guarantee for this code to work properly, nor is there any
 * guarantee for the returned PnO-records to be accurate.
 * Use at your own risk.
 */
#pragma once

#include <string.h>
#include <usb.h>

#include <exception>
#include <vector>

#define BUF_SIZE 64

#define VENDOR_TRAKSTAR2G 0x04b4
#define PRODUCT_TRAKSTAR2G 0x1005

#define ENDPOINT_OUT 0x02
#define ENDPOINT_IN 0x86
#define USB_TIMEOUT 500


// system commands
enum ATC3DGCommands {
	ATC_CMD_XON =				0x11,
	ATC_CMD_XOFF =				0x13,
	ATC_CMD_SELECT =			0x30,
	// 0x3F ???
	ATC_CMD_STREAM =			0x40,
	ATC_CMD_SYNC =				0x41,
	ATC_CMD_POINT =				0x42,
	// 0x43
	// 0x44
	// 0x45
	ATC_CMD_RUN =				0x46,
	ATC_CMD_SLEEP =				0x47,
	ATC_CMD_REFERENCE_FRAME1 =	0x48,
	// 0x49
	ATC_CMD_ANGLE_ALIGN1 =		0x4A,
	// 0x4B
	// 0x4C
	ATC_CMD_BUTTON_MODE =		0x4D,
	ATC_CMD_BUTTON =			0x4E,
	ATC_CMD_EXAMINE =			0x4F,
	ATC_CMD_CHANGE =			0x50,
	// 0x51
	// 0x52
	// 0x53
	// 0x54
	// 0x55
	ATC_CMD_POS =				0x56,
	ATC_CMD_ANG =				0x57,
	ATC_CMD_MAT =				0x58,
	ATC_CMD_POS_ANG =			0x59,
	ATC_CMD_POS_MAT =			0x5A,
	// 0x5B
	ATC_CMD_QUAT =				0x5C,
	ATC_CMD_POS_QUAT =			0x5D,
	// 0x5E
	// 0x5F
	// 0x60
	// 0x61
	ATC_CMD_RESET =				0x62,
	// 0x63
	// 0x64
	// 0x65
	// 0x66
	// 0x67
	// 0x68
	// 0x69
	// 0x6A
	// 0x6B
	// 0x6C
	ATC_CMD_MODELSTRING =		0x6D,
	ATC_CMD_REFERENCE_FRAME	=	0x72,
	ATC_CMD_METAL =				0x73,
	ATC_CMD_BORESIGHT =			0x75, // command not implemented
	ATC_CMD_BORESIGHT_REMOVE =	0x76, // command not implemented
	ATC_CMD_READ_ROM =			0x7B
};

// system parameters
enum ATC3DGParameters {
	ATC_STATUS =				0x00,
	ATC_REVISION =				0x01,
	ATC_SPEED =					0x02,
	ATC_POSITION_SCALING = 		0x03,
	ATC_FILTER =				0x04,
	ATC_DC_ALPHA_MIN =			0x05,
	ATC_RATE_COUNT =			0x06,
	ATC_RATE =					0x07,
	ATC_DATA_READY =			0x08,
	ATC_DATA_READY_CHAR =		0x09,
	ATC_ERROR_CODE =			0x0A,
	ATC_ERROR_BEHAVIOR =		0x0B,
	ATC_DC_VM =					0x0C,
	ATC_DC_ALPHA_MAX =			0x0D,
	ATC_SUDDENCHANGE =			0x0E,
	ATC_IDENTIFICATION =		0x0F,
	ATC_SYSTEM_ERROR =			0x10,
	ATC_REFERENCE_FRAME =		0x11,
	ATC_TX_MODE =				0x12,
	ATC_ADDRESS_MODE =			0x13,
	ATC_POWERLINE_FREQ =		0x14,
	ATC_ADDRESS =				0x15,
	ATC_HEMISPHERE =			0x16,
	ATC_ANGLE_ALIGN2 =			0x17,
	ATC_REFERENCE_FRAME2 =		0x18,
	ATC_SERIAL_NUMBER =			0x19,
	ATC_RX_SERIAL_NUMBER =		0x1A,
	ATC_TX_SERIAL_NUMBER =		0x1B,
	ATC_DELAY =					0x20,
	ATC_GROUP_MODE =			0x23,
	ATC_TRACKER_STATUS =		0x24,
	ATC_AUTOCONFIG =			0x32,
	ATC_STREAM =				0x64,
	ATC_ROM =					0x79
};


class ATC3DGTracker {
public:
	ATC3DGTracker();
	ATC3DGTracker(ATC3DGTracker& t);
	virtual ~ATC3DGTracker();
	
	virtual void connect();
	virtual void update(
		int sensor,
		double&  x, double&  y, double&  z,
		double& ax, double& ay, double& az,
		double* matrix,
		double& q0, double& qi, double& qj, double& qk,
		double& quality,
		bool& button
	);
	virtual void disconnect();
	
	virtual int get_number_sensors();
	
	virtual void set_rate(double rate);
	virtual double get_rate();
	virtual double get_min_rate() const;
	virtual double get_max_rate() const;
	
	virtual bool good() const;

private:
	void p_read(int bytes);
	void p_write(std::vector<int> list);
	double p_get_double(int byte1, int byte2);
	
	void atc_init();
	void atc_select_tx(int tx, int delay=7000);
	void atc_get_status();
	void atc_autoconfig(int units = 0x04, int delay=600);
	void atc_reset(int delay = 6000);
	void atc_sleep(int delay = 6000);
	std::string atc_get_modelstring_pcb();
	std::string atc_get_partnum_pcb();
	std::string atc_get_modelstring_tx();
	std::string atc_get_partnum_tx();
	std::string atc_get_modelstring_rx(int rx);
	std::string atc_get_partnum_rx(int rx);
	
	double m_scaling;
	double m_rate;
	bool m_good;
	
	struct usb_device* m_device;
	struct usb_dev_handle* m_handle;
	
	char m_output_buf[BUF_SIZE];
	char m_input_buf[BUF_SIZE];
};

