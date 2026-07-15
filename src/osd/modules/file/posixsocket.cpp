// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, Vas Crabb
//============================================================
//
//  sdlsocket.c - SDL socket (inet, unix domain) access
//  functions
//
//  SDLMAME by Olivier Galibert and R. Belmont
//
//============================================================

#include "posixfile.h"

#include "osdcore.h"

#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>


namespace {

char const *const posixfile_socket_identifier  = "socket.";
char const *const posixfile_domain_identifier  = "domain.";


class posix_osd_socket : public osd_file
{
public:
	posix_osd_socket(posix_osd_socket const &) = delete;
	posix_osd_socket(posix_osd_socket &&) = delete;
	posix_osd_socket& operator=(posix_osd_socket const &) = delete;
	posix_osd_socket& operator=(posix_osd_socket &&) = delete;

	posix_osd_socket(int sock, bool listening) noexcept
		: m_sock(sock)
		, m_listening(listening)
	{
		assert(m_sock >= 0);
	}

	virtual ~posix_osd_socket()
	{
		::close(m_sock);
	}

	virtual std::error_condition read(void *buffer, std::uint64_t offset, std::uint32_t count, std::uint32_t &actual) noexcept override
	{
		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(m_sock, &readfds);

		struct timeval timeout;
		timeout.tv_sec = timeout.tv_usec = 0;

		if (::select(m_sock + 1, &readfds, nullptr, nullptr, &timeout) < 0)
		{
			return std::error_condition(errno, std::generic_category());
		}
		else if (FD_ISSET(m_sock, &readfds))
		{
			if (!m_listening)
			{
				// connected socket
				ssize_t const result = ::read(m_sock, buffer, count);
				if (result < 0)
				{
					return std::error_condition(errno, std::generic_category());
				}
				else
				{
					actual = std::uint32_t(size_t(result));
					return std::error_condition();
				}
			}
			else
			{
				// listening socket
				int const accepted = ::accept(m_sock, nullptr, nullptr);
				if (accepted < 0)
				{
					return std::error_condition(errno, std::generic_category());
				}
				else
				{
					::close(m_sock);
					m_sock = accepted;
					m_listening = false;
					actual = 0;

					return std::error_condition();
				}
			}
		}
		else
		{
			// no data available
			actual = 0;
			return std::errc::operation_would_block;
		}
	}

	virtual std::error_condition write(void const *buffer, std::uint64_t offset, std::uint32_t count, std::uint32_t &actual) noexcept override
	{
		if (!m_listening)
		{
			// connected socket
			ssize_t const result = ::write(m_sock, buffer, count);
			if (result < 0)
				return std::error_condition(errno, std::generic_category());

			actual = std::uint32_t(size_t(result));
			return std::error_condition();
		}
		else
		{
			// listening socket - writing may raise SIGPIPE
			actual = 0;
			return std::errc::not_connected;
		}
	}

	virtual std::error_condition truncate(std::uint64_t offset) noexcept override
	{
		// doesn't make sense on socket
		return std::errc::bad_file_descriptor;
	}

	virtual std::error_condition flush() noexcept override
	{
		// there's no simple way to flush buffers on a socket anyway
		return std::error_condition();
	}

private:
	int     m_sock;
	bool    m_listening;
};


template <typename T, typename MakeSocket>
std::error_condition create_socket(T const &sa, int sock, std::uint32_t openflags, osd_file::ptr &file, std::uint64_t &filesize, MakeSocket &&make_socket) noexcept
{
	osd_file::ptr result;
	if (openflags & OPEN_FLAG_CREATE)
	{
		// listening socket support
		// bind socket...
		if (::bind(sock, reinterpret_cast<struct sockaddr const *>(&sa), sizeof(sa)) < 0)
		{
			std::error_condition binderr(errno, std::generic_category());
			::close(sock);
			return binderr;
		}

		// start to listen...
		if (::listen(sock, 1) < 0)
		{
			std::error_condition lstnerr(errno, std::generic_category());
			::close(sock);
			return lstnerr;
		}

		// mark socket as "listening"
		result.reset(new (std::nothrow) posix_osd_socket(sock, true));
	}
	else
	{
		// Heathkit HERO fork (R6): a client socket connect gets a bounded
		// (deadline-based: refusal latency is host-dependent — some filter
		// drivers hold a loopback RST for ~2 s) retry with backoff. A single
		// attempt turned a transiently refused/late peer (the harness serial
		// listener) into a permanently byteless null_modem session via the
		// image manager's silent CREATE fallback. After the budget the
		// failure is loud and carries the real error.
		constexpr std::int64_t CONNECT_RETRY_BUDGET_NS = 10'000'000'000;
		constexpr useconds_t CONNECT_RETRY_DELAY_US = 250'000;
		auto const monotonic_ns = [] () -> std::int64_t
		{
			struct timespec ts;
			::clock_gettime(CLOCK_MONOTONIC, &ts);
			return (std::int64_t(ts.tv_sec) * 1'000'000'000) + ts.tv_nsec;
		};
		std::int64_t const retry_deadline = monotonic_ns() + CONNECT_RETRY_BUDGET_NS;
		int attempt = 0;
		for (;;)
		{
			if (::connect(sock, reinterpret_cast<struct sockaddr const *>(&sa), sizeof(sa)) >= 0)
				break;
			int const err = errno;
			::close(sock);
			++attempt;
			bool const retryable =
					(ECONNREFUSED == err) ||
					(ETIMEDOUT == err) ||
					(EADDRINUSE == err) ||
					(ENETUNREACH == err) ||
					(EHOSTUNREACH == err);
			if (!retryable || (monotonic_ns() >= retry_deadline))
			{
				std::error_condition const connerr(err, std::generic_category());
				osd_printf_error("Socket connect failed after %1$d attempt(s): %2$s\n", attempt, connerr.message());
				return connerr;
			}
			::usleep(CONNECT_RETRY_DELAY_US);
			sock = make_socket();
			if (sock < 0)
				return std::error_condition(errno, std::generic_category());
		}
		result.reset(new (std::nothrow) posix_osd_socket(sock, false));
	}

	if (!result)
	{
		::close(sock);
		return std::errc::not_enough_memory;
	}
	file = std::move(result);
	filesize = 0;
	return std::error_condition();
}

} // anonymous namespace


/*
    Checks whether the path is a socket specification. A valid socket
    specification has the format "socket." host ":" port. Host may be simple
    or fully qualified. Port must be between 1 and 65535.
*/
bool posix_check_socket_path(std::string const &path) noexcept
{
	if (strncmp(path.c_str(), posixfile_socket_identifier, strlen(posixfile_socket_identifier)) == 0 &&
		strchr(path.c_str(), ':') != nullptr) return true;
	return false;
}


bool posix_check_domain_path(std::string const &path) noexcept
{
	if (strncmp(path.c_str(), posixfile_domain_identifier, strlen(posixfile_domain_identifier)) == 0)
		return true;
	return false;
}


std::error_condition posix_open_socket(std::string const &path, std::uint32_t openflags, osd_file::ptr &file, std::uint64_t &filesize) noexcept
{
	char hostname[256];
	int port;
	std::sscanf(&path[strlen(posixfile_socket_identifier)], "%255[^:]:%d", hostname, &port);

	struct hostent const *const localhost = ::gethostbyname(hostname);
	if (!localhost)
		return std::errc::no_such_file_or_directory;

	struct sockaddr_in sai;
	memset(&sai, 0, sizeof(sai));
	sai.sin_family = AF_INET;
	sai.sin_port = htons(port);
	sai.sin_addr = *reinterpret_cast<struct in_addr *>(localhost->h_addr);

	auto const make_socket = [] () -> int
	{
		int const s = ::socket(AF_INET, SOCK_STREAM, 0);
		if (s < 0)
			return -1;
		int const flag = 1;
		if ((::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&flag), sizeof(flag)) < 0) ||
			(::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&flag), sizeof(flag)) < 0))
		{
			int const sockopterr = errno;
			::close(s);
			errno = sockopterr;
			return -1;
		}
		return s;
	};

	int const sock = make_socket();
	if (sock < 0)
		return std::error_condition(errno, std::generic_category());

	return create_socket(sai, sock, openflags, file, filesize, make_socket);
}


std::error_condition posix_open_domain(std::string const &path, std::uint32_t openflags, osd_file::ptr &file, std::uint64_t &filesize) noexcept
{
	struct sockaddr_un sau;
	memset(&sau, 0, sizeof(sau));
	sau.sun_family = AF_UNIX;
	strncpy(sau.sun_path, &path.c_str()[strlen(posixfile_domain_identifier)], sizeof(sau.sun_path)-1);

	auto const make_socket = [] () -> int
	{
		int const s = ::socket(AF_UNIX, SOCK_STREAM, 0);
		if (s < 0)
			return -1;
		if (fcntl(s, F_SETFL, O_NONBLOCK) < 0)
		{
			int const cntlerr = errno;
			::close(s);
			errno = cntlerr;
			return -1;
		}
		return s;
	};

	int const sock = make_socket();
	if (sock < 0)
		return std::error_condition(errno, std::generic_category());

	return create_socket(sau, sock, openflags, file, filesize, make_socket);
}
