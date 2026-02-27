/** Serial.cpp
 *
 * A very simple serial port control class that does NOT require MFC/AFX.
 *
 * @author Hans de Ruiter
 *
 * @version 0.1 -- 28 October 2008
 */
 //#include "stdafx.h"
#include <iostream>
#include <mutex>
#include <windows.h>  // For Sleep function
#define MAX_RETRY 3
using namespace std;

#include "Serial.h"

static uint16_t  Mavis_CRC(char message_new[30], int len)
{
	uint16_t crc = 0xFFFF;
	for (int pos = 0; pos < len; pos++) {
		uint8_t last_byte = message_new[pos];
		crc ^= (uint16_t)(last_byte);          // XOR byte into least sig. byte of crc
		for (int i = 8; i != 0; i--) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                          // Else LSB is not set
				crc >>= 1;                    // Just shift right
		}
	}
	return crc;
}

Serial::Serial(tstring& commPortName, int bitRate)
{
	if (!commPortName.empty()) {
		commHandle = CreateFile(commPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, //
			0, NULL);

		if (commHandle == INVALID_HANDLE_VALUE)
		{
			DWORD error = GetLastError();
			std::cerr << error << endl;// "Failed to open";
			throw("ERROR: Could not open com port");
		}
		else
		{
			// set timeouts
			COMMTIMEOUTS cto = { MAXDWORD, 0, 30, 0, 30 };
			DCB dcb;
			if (!SetCommTimeouts(commHandle, &cto))
			{
				Serial::~Serial();
				throw("ERROR: Could not set com port time-outs");
			}

			// set DCB
			memset(&dcb, 0, sizeof(dcb));
			dcb.DCBlength = sizeof(dcb);
			dcb.BaudRate = bitRate;
			dcb.fBinary = 1;
			dcb.fDtrControl = DTR_CONTROL_ENABLE;
			dcb.fRtsControl = RTS_CONTROL_ENABLE;

			dcb.Parity = NOPARITY;
			dcb.StopBits = ONESTOPBIT;
			dcb.ByteSize = 8;

			if (!SetCommState(commHandle, &dcb))
			{
				Serial::~Serial();
				throw("ERROR: Could not set com port parameters");
			}
		}
	}
}

Serial::~Serial()
{
	CloseHandle(commHandle);
}

int Serial::write(const char* buffer)
{
	DWORD numWritten;
	WriteFile(commHandle, buffer, strlen(buffer), &numWritten, NULL);

	return numWritten;
}

int Serial::write(const char* buffer, int buffLen)
{
	DWORD numWritten;
	WriteFile(commHandle, buffer, buffLen, &numWritten, NULL);

	return numWritten;
}

int Serial::MavisSendComData(std::mutex* comm_lock, int code, int data)
{
	std::unique_lock<std::mutex> lock(*comm_lock);
	int temp1, temp2, temp3, temp4;
	char buffer[7];
	int numRead, numWriten;
	int retry_cnt = 0;
	while (retry_cnt < MAX_RETRY) {
		buffer[0] = 255;
		buffer[1] = code + 128;
		buffer[2] = temp1 = data & 0x0F;
		buffer[3] = temp2 = (data & 0xF0) >> 4;
		buffer[4] = temp3 = (data & 0xF00) >> 8;
		buffer[5] = temp4 = (data & 0xF000) >> 12;
		uint16_t  check_value = Mavis_CRC(buffer, 6);
		uint8_t  partA = (uint8_t)((check_value & 0xFF00) >> 8);
		uint8_t  partB = (uint8_t)(check_value & 0x00FF);
		buffer[6] = partA & 0x7F;
		numWriten = write(buffer, 7);
		Sleep(4);
		numRead = read(buffer, 6, false);
		if (numRead == 6) {
			if ((buffer[1] == 10) && (buffer[2] == 10)) return 0;
			else std::cout << "retry " << retry_cnt + 1 << endl;
		}
		std::cout << "error " << endl;
	}
	return -1;
}

int Serial::MavisGetComData(std::mutex* comm_lock, int code, int* data)
{
	std::unique_lock<std::mutex> lock(*comm_lock);
	int checksum = 0;
	int local_data;
	char buffer[7];
	int numRead, numWriten;
	int retry_cnt = 0;
	while (retry_cnt < MAX_RETRY) {
		buffer[0] = 255;
		buffer[1] = code + 192;
		uint16_t  check_value = Mavis_CRC(buffer, 2);
		uint8_t  partA = (uint8_t)((check_value & 0xFF00) >> 8);
		uint8_t  partB = (uint8_t)(check_value & 0x00FF);
		buffer[2] = partA & 0x7F;
		numWriten = write(buffer, 3);
		Sleep(4);
		numRead = read(buffer, 6, false);
		local_data = buffer[4];
		local_data = (local_data << 4) | buffer[3];
		local_data = (local_data << 4) | buffer[2];
		local_data = (local_data << 4) | buffer[1];
		check_value = Mavis_CRC(buffer, 5);
		partA = (uint8_t)((check_value & 0xFF00) >> 8);
		partB = (uint8_t)(check_value & 0x00FF);
		checksum = partA & 0x7F;
		if (numRead == 6) {
			if (buffer[5] == checksum) {
				*data = local_data;
				return 0;
			}
			else {
				std::cout << "retry " << retry_cnt + 1 << endl;
			}
		}
		retry_cnt++;
	}
	std::cout << "error " << endl;
	return -1;
}

int Serial::read(char* buffer, int buffLen, bool nullTerminate)
{
	DWORD numRead;
	if (nullTerminate)
	{
		--buffLen;
	}

	BOOL ret = ReadFile(commHandle, buffer, buffLen, &numRead, NULL);

	if (!ret)
	{
		return 0;
	}

	if (nullTerminate)
	{
		buffer[numRead] = '\0';
	}

	return numRead;
}

#define FLUSH_BUFFSIZE 10

void Serial::flush()
{
	char buffer[FLUSH_BUFFSIZE];
	int numBytes = read(buffer, FLUSH_BUFFSIZE, false);
	while (numBytes != 0)
	{
		numBytes = read(buffer, FLUSH_BUFFSIZE, false);
	}
}
