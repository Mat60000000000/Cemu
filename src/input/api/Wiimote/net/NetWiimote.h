#pragma once

#include "input/api/Wiimote/WiimoteDevice.h"

// Network transport for Wiimotes: instead of talking to a local Bluetooth/HID Wiimote, the raw
// Wiimote HID reports are exchanged over UDP with a small bridge program running on the machine
// where the real Wiimote is connected (see tools/wiimote_bridge.py).
//
// Disabled unless the environment variable CEMU_NETWIIMOTE is set:
//   CEMU_NETWIIMOTE=<bind ip>:<port>   e.g. 192.168.27.1:26790   (or just <port> to listen on all interfaces)
//
// Datagram layout (little endian):
//   'W' 'M' | slot (u8, 0..3) | kind (u8) | seq (u16) | payload
//   kind 1: Wiimote -> Cemu, raw input report     (payload = report id + data, as read from the HID device)
//   kind 2: Wiimote -> Cemu, keepalive             (no payload)
//   kind 3: Cemu -> Wiimote, raw output report     (payload = report id + data, as written to the HID device)
class NetWiimote : public WiimoteDevice
{
public:
	explicit NetWiimote(uint8 slot);

	bool write_data(const std::vector<uint8>& data) override;
	std::optional<std::vector<uint8>> read_data() override;
	bool operator==(const WiimoteDevice& o) const override;

	static std::vector<WiimoteDevicePtr> get_devices();

private:
	uint8 m_slot;
};
