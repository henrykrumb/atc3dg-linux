#include <chrono>
#include <exception>
#include <thread>
#include <sstream>
#include <iostream>
#include <cmath>

#include "atc3dg.hpp"

void log_debug(std::string string)
{
#ifndef NDEBUG
	std::cout << "[DEBUG] " << string << std::endl;
#endif
}

ATC3DGTracker::ATC3DGTracker() : m_scaling(1),
								 m_rate(80),
								 m_good(false)
{
}

ATC3DGTracker::~ATC3DGTracker()
{
	if (m_good)
	{
		disconnect();
	}
}

void ATC3DGTracker::connect()
{
	log_debug("connecting to ATC 3D Guidance tracker");
	usb_init();
	usb_find_busses();
	usb_find_devices();

	struct usb_bus *bus = nullptr;
	struct usb_device *dev = nullptr;

	for (bus = usb_busses; bus; bus = bus->next)
	{
		for (dev = bus->devices; dev; dev = dev->next)
		{
			int vendor = dev->descriptor.idVendor;
			int product = dev->descriptor.idProduct;
			if (vendor == VENDOR_TRAKSTAR2G && product == PRODUCT_TRAKSTAR2G)
			{
				m_device = dev;
				break;
			}
		}
	}

	if (m_device == nullptr)
	{
		throw std::runtime_error("Could not find USB device.");
	}

	m_handle = usb_open(m_device);
	if (!m_handle)
	{
		throw std::runtime_error("Could not open USB device.");
	}

	int status = usb_set_configuration(m_handle, 1);
	if (status < 0)
	{
		throw std::runtime_error("Could not set USB configuration.");
	}

	status = usb_claim_interface(m_handle, 0);
	if (status < 0)
	{
		throw std::runtime_error("Could not claim USB interface.");
	}

	status = usb_set_altinterface(m_handle, 0);
	if (status < 0)
	{
		throw std::runtime_error("Could not set altinterface.");
	}

	status = usb_clear_halt(m_handle, ENDPOINT_IN);
	if (status < 0)
	{
		throw std::runtime_error("Clear halt failed.");
	}

	// clear pipe
	usb_bulk_read(m_handle, ENDPOINT_IN, m_input_buf, 64, USB_TIMEOUT);

	log_debug("Initializing trakSTAR unit...");
	atc_init();
	log_debug("Connected to trakSTAR 3D Guidance tracker.");
	m_good = true;
}

void ATC3DGTracker::disconnect()
{
	m_good = false;
	log_debug("Disconnecting trakSTAR 3D Guidance tracker...");
	atc_select_tx(0xFF);
	p_write({0x3F});
	p_write({0x3F});
	p_write({0xF1, ATC_CMD_CHANGE, ATC_GROUP_MODE, 0x00});
	p_write({ATC_CMD_CHANGE, 0x94, 0x01});
	atc_sleep(0);
	usb_close(m_handle);
}

int ATC3DGTracker::get_number_sensors()
{
	int n_sensors = 0;
	for (int i = 0; i < 4; i++)
	{
		p_write({0xF1 + i, ATC_CMD_EXAMINE, ATC_RX_SERIAL_NUMBER});
		p_read(2);
		if (m_input_buf[0] != 0 && m_input_buf[1] != 0)
		{
			printf("%x%x\n", m_input_buf[0], m_input_buf[1]);
			n_sensors++;
		}
	}
	return n_sensors;
}

double ATC3DGTracker::p_get_double(int byte1, int byte2)
{
	if (byte2 < 0) {
		byte2 = byte1 + 1;
	}
	short v = ((m_input_buf[byte2] << 7) | (m_input_buf[byte1] & 0x7F)) << 2;
	return (double)v / 0x8000;
}

void ATC3DGTracker::update(
	int sensor,
	double &x, double &y, double &z,
	double &ax, double &ay, double &az,
	double (*matrix)[3][3],
	double &q0, double &qi, double &qj, double &qk,
	double &quality,
	bool &button)
{
	if (!m_good)
	{
		return;
	}
	button = false;

	p_write({0xF1 + sensor, ATC_CMD_POINT});
	p_read(53);

	// positions
	x = 36.0 * m_scaling * p_get_double(0) * 2.54;
	y = 36.0 * m_scaling * p_get_double(2) * 2.54;
	z = 36.0 * m_scaling * p_get_double(4) * 2.54;
	// angles
	ax = 180.0 * p_get_double(6);
	ay = 180.0 * p_get_double(8);
	az = 180.0 * p_get_double(10);

	double d2r = M_PI / 180.0;
	*matrix[0][0] =  cos(d2r * ay) * cos(d2r * az);
	*matrix[0][1] =  cos(d2r * ay) * sin(d2r * az);
	*matrix[0][2] = -sin(d2r * ay);
	*matrix[1][0] = -(cos(d2r * ax) * sin(d2r * az)) + (sin(d2r * ax) * sin(d2r * ay) * cos(d2r * az));
	*matrix[1][1] =  (cos(d2r * ax) * cos(d2r * az)) + (sin(d2r * ax) * sin(d2r * ay) * sin(d2r * az));
	*matrix[1][2] =  sin(d2r * ax) * cos(d2r * ay);
	*matrix[2][0] =  (sin(d2r * ax) * sin(d2r * az)) + (cos(d2r * ax) * sin(d2r * ay) * cos(d2r * az));
	*matrix[2][1] = -(sin(d2r * ax) * cos(d2r * az)) + (cos(d2r * ax) * sin(d2r * ay) * sin(d2r * az));
	*matrix[2][2] =  cos(d2r * ax) * cos(d2r * ay);

	// matrix
	/*int offset = 12;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int idx = i * 3 + j;
			*matrix[i][j] = p_get_double(offset + idx * 2);
		}
		std::cout << std::endl;
	}
	*/

	q0 = p_get_double(30);
	qi = p_get_double(32);
	qj = p_get_double(34);
	qk = p_get_double(36);

	// quality
	quality = p_get_double(36, 37);
	// timestamp
	// TODO @henry EMTS timestamp [44:51]

	if ((m_input_buf[52] & 1) == 1)
	{
		button = true;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void ATC3DGTracker::update(
	int sensor,
	double *position,
	double *orientation,
	double (*matrix)[3][3],
	double *quaternion,
	double *quality,
	bool *button)
{
	this->update(
		sensor,
		position[0], position[1], position[2],
		orientation[0], orientation[1], orientation[2],
		matrix,
		quaternion[0], quaternion[1], quaternion[2], quaternion[3],
		*quality,
		*button);
}

void ATC3DGTracker::set_rate(double rate)
{
	if (rate >= get_min_rate() && rate <= get_max_rate())
	{
		m_rate = rate;
		short s_rate = (short)(rate * 256);
		p_write({ATC_CMD_CHANGE, ATC_RATE, s_rate & 0xFF, s_rate >> 8});
	}
}

double ATC3DGTracker::get_rate() const
{
	return m_rate;
}

double ATC3DGTracker::get_min_rate() const
{
	return 20;
}

double ATC3DGTracker::get_max_rate() const
{
	return 255;
}

bool ATC3DGTracker::good() const
{
	return m_good;
}

/** -=-=-= trakSTAR interface functions =-=-=- **/

void ATC3DGTracker::atc_init()
{
	log_debug("starting initialization routine");
	p_write({0xF1, ATC_CMD_CHANGE, ATC_GROUP_MODE, 0x00});
	p_write({0x3F});
	p_write({0x3F});
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	p_write({0xF1, ATC_CMD_EXAMINE, 0x46});
	p_read(1);

	atc_reset(700);
	atc_sleep(6000);

	p_write({0xF1, ATC_CMD_EXAMINE, 0x46});
	p_read(1);

	p_write({0x3F});

	p_write({0xF1, ATC_CMD_CHANGE, 0x64, 0x01});

	p_write({0xF1, ATC_CMD_CHANGE, 0x7B, 0x00, 0x00});
	p_write({0xF1, ATC_CMD_EXAMINE, 0x7B});
	p_read(2);

	p_write({0xF1, ATC_CMD_CHANGE, 0x7B, 0x01, 0x00});
	p_write({0xF1, ATC_CMD_EXAMINE, 0x7B});
	p_read(2);

	for (int j = 0; j < 9; j++)
	{
		for (int i = 0; i < 9; i++)
		{
			p_write({0xF1, ATC_CMD_CHANGE, 0x7B, 0x01 + i, 0x00 + j});
			p_write({0xF1, ATC_CMD_EXAMINE, ATC_ROM});
			p_read(32);
			p_read(32);
		}
	}

	p_write({0xF1, ATC_CMD_EXAMINE, ATC_POSITION_SCALING});
	p_read(2);

	m_scaling = 1;
	if (m_input_buf[0] == 1)
	{
		m_scaling = 2;
	}

	p_write({0xF1, ATC_CMD_EXAMINE, 0x94});
	p_read(1);

	p_write({0xF1, ATC_CMD_CHANGE, 0x94, 0x01});
	p_write({0xF1, ATC_CMD_CHANGE, 0x64, 0x01});
	atc_sleep(0);

	p_write({0xF1, ATC_CMD_EXAMINE, 0x95});
	p_read(1);

	p_write({0xF1, ATC_CMD_EXAMINE, 0x34});
	p_read(2);

	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	p_write({0xF1, ATC_CMD_CHANGE, 0x29, 0x00, 0x00, 0x5F, 0xA2, 0x58, 0x5A});

	atc_autoconfig(0);
	atc_sleep(600);

	p_write({0xF1, ATC_CMD_EXAMINE, ATC_AUTOCONFIG});
	p_read(5);

	p_write({0xF1, ATC_CMD_EXAMINE, 0x46});
	p_read(1);

	p_write({0x7A});

	p_write({0xF1, ATC_CMD_EXAMINE, ATC_SERIAL_NUMBER});
	p_read(2);

	atc_get_modelstring_pcb();
	atc_get_partnum_pcb();
	atc_get_modelstring_tx();
	atc_get_partnum_tx();

	p_write({0x3F});

	atc_select_tx(0);

	for (int rx = 0; rx < 4; rx++)
	{
		p_write({0xF1 + rx, 0x5F});
		p_write({0xF1 + rx, ATC_CMD_CHANGE, 0x80, 0x01});
	}

	p_write({0xF1, ATC_CMD_CHANGE, ATC_STREAM, 0x01});
	p_write({0xF1, ATC_CMD_EXAMINE, ATC_SYSTEM_ERROR});
	p_read(2);
	p_write({0xF1, ATC_CMD_EXAMINE, 0x7A});
	p_read(32);
	p_write({0xF1, ATC_CMD_EXAMINE, ATC_SYSTEM_ERROR});
	p_read(2);
	p_write({0xF1, ATC_CMD_CHANGE, ATC_STREAM, 0x00});

	p_write({ATC_CMD_EXAMINE, 0});
	p_read(2);
	printf("%x %x\n", m_input_buf[0], m_input_buf[1]);
}

void ATC3DGTracker::atc_select_tx(int tx, int delay)
{
	p_write({ATC_CMD_SELECT, tx});
	std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

void ATC3DGTracker::atc_get_status()
{
	p_write({0xF1, ATC_CMD_EXAMINE, ATC_STATUS});
	p_read(2);
}

std::string ATC3DGTracker::atc_get_modelstring_pcb()
{
	std::stringstream strstr;

	for (int b = 0; b < 11; b++)
	{
		p_write({ATC_CMD_MODELSTRING, 0x0A + b, 0x00});
		p_read(1);
		if (m_input_buf[0] >= ' ')
		{
			strstr << m_input_buf[0];
		}
	}

	return strstr.str();
}

std::string ATC3DGTracker::atc_get_partnum_pcb()
{
	std::stringstream strstr;

	for (int b = 0; b < 15; b++)
	{
		p_write({ATC_CMD_MODELSTRING, 0x70 + b, 0x00});
		p_read(1);
		if (m_input_buf[0] >= ' ')
		{
			strstr << m_input_buf[0];
		}
	}

	return strstr.str();
}

std::string ATC3DGTracker::atc_get_modelstring_tx()
{
	std::stringstream strstr;

	for (int b = 0; b < 11; b++)
	{
		p_write({ATC_CMD_MODELSTRING, 0x0A + b, 0x02});
		p_read(1);
		if (m_input_buf[0] >= ' ')
		{
			strstr << m_input_buf[0];
		}
	}

	return strstr.str();
}

std::string ATC3DGTracker::atc_get_partnum_tx()
{
	std::stringstream strstr;

	for (int b = 0; b < 15; b++)
	{
		p_write({ATC_CMD_MODELSTRING, 0x70 + b, 0x02});
		p_read(1);
		if (m_input_buf[0] >= ' ')
		{
			strstr << m_input_buf[0];
		}
	}

	return strstr.str();
}

std::string ATC3DGTracker::atc_get_modelstring_rx(int rx)
{
	std::stringstream strstr;

	for (int b = 0; b < 11; b++)
	{
		p_write({0xF1 + rx, ATC_CMD_MODELSTRING, 0x0A + b, 0x01});
		p_read(1);
		if (m_input_buf[0] >= ' ')
		{
			strstr << m_input_buf[0];
		}
	}

	return strstr.str();
}

std::string ATC3DGTracker::atc_get_partnum_rx(int rx)
{
	std::stringstream strstr;

	for (int b = 0; b < 15; b++)
	{
		p_write({0xF1 + rx, ATC_CMD_MODELSTRING, 0x70 + b, 0x01});
		p_read(1);
		if (m_input_buf[0] >= ' ')
		{
			strstr << m_input_buf[0];
		}
	}

	return strstr.str();
}

void ATC3DGTracker::atc_autoconfig(int units, int delay)
{
	p_write({0xF1, ATC_CMD_CHANGE, ATC_AUTOCONFIG, units});
	std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

void ATC3DGTracker::atc_reset(int delay)
{
	p_write({ATC_CMD_RESET});
	std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

void ATC3DGTracker::atc_sleep(int delay)
{
	p_write({ATC_CMD_SLEEP});
	std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

/**
 * \param bytes number of bytes to read
 */
void ATC3DGTracker::p_read(int bytes)
{
	int r = 0;

	if (bytes >= BUF_SIZE)
	{
		throw std::runtime_error("Tried to read more than 64 bytes from USB. This is a bug.");
	}

	do
	{
		r = usb_bulk_read(m_handle, ENDPOINT_IN, m_input_buf, bytes, USB_TIMEOUT);
	} while (r == 0);

	if (r != bytes)
	{
		fprintf(stderr, "Attempted to read %d bytes, read %d.\n", bytes, r);
		throw std::runtime_error(usb_strerror());
	}

	m_input_buf[bytes] = '\0';
}

/**
 * \param list vector of arguments
 */
void ATC3DGTracker::p_write(std::vector<int> list)
{
	int length = 0;
	for (auto it : list)
	{
		m_output_buf[length] = it;
		length++;
	}

	int r = 0;

	do
	{
		r = usb_bulk_write(m_handle, ENDPOINT_OUT, m_output_buf, length, USB_TIMEOUT);
	} while (r == 0);

	if (r != length)
	{
		fprintf(stderr, "Attempted to write %d bytes, read %d.\n", length, r);
		throw std::runtime_error(usb_strerror());
	}
}
