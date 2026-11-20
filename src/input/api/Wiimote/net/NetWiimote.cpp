#include "input/api/Wiimote/net/NetWiimote.h"
#include "util/Socket/Socket.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>

namespace
{
	constexpr size_t kMaxSlots = 4;
	constexpr size_t kHeaderSize = 6;
	constexpr size_t kMaxQueuedReports = 512;
	constexpr auto kAliveTimeout = std::chrono::seconds(3);

	enum Kind : uint8
	{
		kInputReport = 1,
		kKeepAlive = 2,
		kOutputReport = 3,
	};

	class NetWiimoteServer
	{
	public:
		static NetWiimoteServer* Get()
		{
			static std::unique_ptr<NetWiimoteServer> s_instance = Create();
			return s_instance.get();
		}

		~NetWiimoteServer()
		{
			m_running = false;
			m_waiter.InterruptWait();
			if (m_thread.joinable())
				m_thread.join();
		}

		bool IsAlive(uint8 slot)
		{
			if (slot >= kMaxSlots)
				return false;
			std::scoped_lock lock(m_slots[slot].mutex);
			return IsAliveLocked(m_slots[slot]);
		}

		std::optional<std::vector<uint8>> Read(uint8 slot)
		{
			auto& s = m_slots[slot];
			std::scoped_lock lock(s.mutex);
			if (!IsAliveLocked(s))
				return std::nullopt; // bridge went away -> report the device as disconnected
			if (s.queue.empty())
				return std::vector<uint8>{};
			auto report = std::move(s.queue.front());
			s.queue.pop_front();
			return report;
		}

		bool Write(uint8 slot, const std::vector<uint8>& data)
		{
			auto& s = m_slots[slot];
			SocketIPv4 ip;
			uint16 port;
			uint16 seq;
			{
				std::scoped_lock lock(s.mutex);
				if (!IsAliveLocked(s))
					return false;
				ip = s.ip;
				port = s.port;
				seq = s.outSeq++;
			}
			std::vector<uint8> packet(kHeaderSize + data.size());
			packet[0] = 'W';
			packet[1] = 'M';
			packet[2] = slot;
			packet[3] = kOutputReport;
			packet[4] = static_cast<uint8>(seq & 0xFF);
			packet[5] = static_cast<uint8>(seq >> 8);
			std::copy(data.begin(), data.end(), packet.begin() + kHeaderSize);

			// UDP gives no delivery guarantee. Wiimote output reports (memory writes, report mode, LEDs, rumble) are
			// idempotent, so each one is sent twice to survive a lost datagram. The bridge drops the duplicate by sequence number.
			std::scoped_lock sendLock(m_sendMutex);
			const bool first = m_socket.SendTo(ip, port, packet.data(), packet.size());
			const bool second = m_socket.SendTo(ip, port, packet.data(), packet.size());
			return first || second;
		}

	private:
		struct Slot
		{
			std::mutex mutex;
			std::deque<std::vector<uint8>> queue;
			SocketIPv4 ip = 0;
			uint16 port = 0;
			bool everSeen = false;
			std::chrono::steady_clock::time_point lastSeen{};
			bool haveSeq = false;
			uint16 lastSeq = 0;
			uint16 outSeq = 0;
		};

		static bool IsAliveLocked(const Slot& s)
		{
			return s.everSeen && (std::chrono::steady_clock::now() - s.lastSeen) < kAliveTimeout;
		}

		static std::unique_ptr<NetWiimoteServer> Create()
		{
			const char* env = std::getenv("CEMU_NETWIIMOTE");
			if (!env || !*env)
				return nullptr;

			std::string value(env);
			std::string host = "0.0.0.0";
			std::string portStr = value;
			if (const auto colon = value.rfind(':'); colon != std::string::npos)
			{
				host = value.substr(0, colon);
				portStr = value.substr(colon + 1);
			}
			const int port = std::atoi(portStr.c_str());
			const auto ip = UDPSocket::Resolve(host);
			if (port <= 0 || port > 65535 || !ip)
			{
				cemuLog_log(LogType::Force, "NetWiimote: invalid CEMU_NETWIIMOTE value '{}' (expected <ip>:<port>)", value);
				return nullptr;
			}

			auto server = std::unique_ptr<NetWiimoteServer>(new NetWiimoteServer());
			if (!server->m_socket.IsValid() || !server->m_socket.Bind(*ip, static_cast<uint16>(port)))
			{
				cemuLog_log(LogType::Force, "NetWiimote: unable to listen on {}", value);
				return nullptr;
			}
			cemuLog_log(LogType::Force, "NetWiimote: listening for network Wiimotes on {}", value);
			server->m_running = true;
			server->m_thread = std::thread([s = server.get()]() { s->ReceiveLoop(); });
			return server;
		}

		NetWiimoteServer() = default;

		void ReceiveLoop()
		{
			SetThreadName("NetWiimote-recv");
			std::array<uint8, 2048> buffer{};
			UDPSocket* sockets[] = {&m_socket};
			std::vector<UDPSocket*> ready;
			while (m_running)
			{
				ready.clear();
				m_waiter.WaitForRecv(sockets, ready, 250);
				SocketIPv4 ip;
				uint16 port;
				while (const auto len = m_socket.RecvFrom(ip, port, buffer.data(), buffer.size()))
				{
					HandleDatagram(ip, port, buffer.data(), *len);
				}
			}
		}

		void HandleDatagram(SocketIPv4 ip, uint16 port, const uint8* data, size_t len)
		{
			if (len < kHeaderSize || data[0] != 'W' || data[1] != 'M')
				return;
			const uint8 slot = data[2];
			const uint8 kind = data[3];
			const uint16 seq = static_cast<uint16>(data[4] | (data[5] << 8));
			if (slot >= kMaxSlots || (kind != kInputReport && kind != kKeepAlive))
				return;

			auto& s = m_slots[slot];
			std::scoped_lock lock(s.mutex);
			const auto now = std::chrono::steady_clock::now();
			if (!IsAliveLocked(s))
			{
				// new or restarted bridge: forget the old sequence numbers and stale data
				s.haveSeq = false;
				s.queue.clear();
				cemuLog_log(LogType::Force, "NetWiimote: slot {} connected", slot);
			}
			s.everSeen = true;
			s.ip = ip;
			s.port = port;
			s.lastSeen = now;

			if (kind != kInputReport)
				return;
			// drop duplicated or reordered datagrams (16 bit sequence numbers, wrap-around aware)
			if (s.haveSeq && static_cast<std::int16_t>(seq - s.lastSeq) <= 0)
				return;
			s.haveSeq = true;
			s.lastSeq = seq;
			if (len > kHeaderSize && s.queue.size() < kMaxQueuedReports)
				s.queue.emplace_back(data + kHeaderSize, data + len);
		}

		UDPSocket m_socket;
		UDPWaiter m_waiter;
		std::mutex m_sendMutex;
		std::array<Slot, kMaxSlots> m_slots;
		std::atomic<bool> m_running{false};
		std::thread m_thread;
	};
}

NetWiimote::NetWiimote(uint8 slot)
	: m_slot(slot)
{
}

bool NetWiimote::write_data(const std::vector<uint8>& data)
{
	auto* server = NetWiimoteServer::Get();
	return server && server->Write(m_slot, data);
}

std::optional<std::vector<uint8>> NetWiimote::read_data()
{
	auto* server = NetWiimoteServer::Get();
	if (!server)
		return std::nullopt;
	return server->Read(m_slot);
}

bool NetWiimote::operator==(const WiimoteDevice& o) const
{
	const auto* other = dynamic_cast<const NetWiimote*>(&o);
	return other && other->m_slot == m_slot;
}

std::vector<WiimoteDevicePtr> NetWiimote::get_devices()
{
	std::vector<WiimoteDevicePtr> devices;
	auto* server = NetWiimoteServer::Get();
	if (!server)
		return devices;
	for (uint8 slot = 0; slot < kMaxSlots; ++slot)
	{
		if (server->IsAlive(slot))
			devices.push_back(std::make_shared<NetWiimote>(slot));
	}
	return devices;
}
