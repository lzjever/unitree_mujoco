#include "reference_frame_client.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <iostream>
#include <limits>
#include <utility>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace agentic_ref
{
namespace
{

bool ParseHttpUrl(const std::string& url, std::string* host, int* port, std::string* path)
{
  constexpr const char* prefix = "http://";
  if (url.compare(0, std::strlen(prefix), prefix) != 0)
  {
    return false;
  }
  std::size_t host_begin = std::strlen(prefix);
  std::size_t path_begin = url.find('/', host_begin);
  std::string authority = path_begin == std::string::npos
      ? url.substr(host_begin)
      : url.substr(host_begin, path_begin - host_begin);
  if (authority.empty())
  {
    return false;
  }

  std::size_t colon = authority.rfind(':');
  if (colon != std::string::npos)
  {
    *host = authority.substr(0, colon);
    try
    {
      *port = std::stoi(authority.substr(colon + 1));
    }
    catch (...)
    {
      return false;
    }
  }
  else
  {
    *host = authority;
    *port = 80;
  }
  *path = path_begin == std::string::npos ? "/" : url.substr(path_begin);
  return !host->empty() && *port > 0 && *port <= 65535;
}

int ConnectWithTimeout(const std::string& host, int port, int timeout_ms)
{
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* result = nullptr;
  const std::string service = std::to_string(port);
  if (getaddrinfo(host.c_str(), service.c_str(), &hints, &result) != 0)
  {
    return -1;
  }

  int sock = -1;
  for (addrinfo* rp = result; rp != nullptr; rp = rp->ai_next)
  {
    sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (sock < 0)
    {
      continue;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0 || fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0)
    {
      close(sock);
      sock = -1;
      continue;
    }

    int rc = connect(sock, rp->ai_addr, rp->ai_addrlen);
    if (rc == 0)
    {
      fcntl(sock, F_SETFL, flags);
      break;
    }
    if (errno != EINPROGRESS)
    {
      close(sock);
      sock = -1;
      continue;
    }

    pollfd pfd{};
    pfd.fd = sock;
    pfd.events = POLLOUT;
    rc = poll(&pfd, 1, timeout_ms);
    if (rc > 0)
    {
      int err = 0;
      socklen_t len = sizeof(err);
      if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0)
      {
        fcntl(sock, F_SETFL, flags);
        break;
      }
    }

    close(sock);
    sock = -1;
  }

  freeaddrinfo(result);
  return sock;
}

bool SendAll(int sock, const std::string& request)
{
  const char* data = request.data();
  std::size_t remaining = request.size();
  while (remaining > 0)
  {
    ssize_t sent = send(sock, data, remaining, MSG_NOSIGNAL);
    if (sent <= 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      return false;
    }
    data += sent;
    remaining -= static_cast<std::size_t>(sent);
  }
  return true;
}

bool IEquals(std::string a, std::string b)
{
  if (a.size() != b.size())
  {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    const auto ca = static_cast<unsigned char>(a[i]);
    const auto cb = static_cast<unsigned char>(b[i]);
    if (std::tolower(ca) != std::tolower(cb))
    {
      return false;
    }
  }
  return true;
}

bool ParseContentLength(const std::string& headers, std::size_t* content_length)
{
  std::size_t line_begin = headers.find("\r\n");
  if (line_begin == std::string::npos)
  {
    return false;
  }
  line_begin += 2;

  while (line_begin < headers.size())
  {
    std::size_t line_end = headers.find("\r\n", line_begin);
    if (line_end == std::string::npos)
    {
      line_end = headers.size();
    }
    std::string line = headers.substr(line_begin, line_end - line_begin);
    std::size_t colon = line.find(':');
    if (colon != std::string::npos && IEquals(line.substr(0, colon), "Content-Length"))
    {
      std::size_t pos = colon + 1;
      while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos])))
      {
        ++pos;
      }
      if (pos == line.size())
      {
        return false;
      }

      std::size_t value = 0;
      for (; pos < line.size(); ++pos)
      {
        unsigned char ch = static_cast<unsigned char>(line[pos]);
        if (std::isspace(ch))
        {
          while (pos < line.size() && std::isspace(static_cast<unsigned char>(line[pos])))
          {
            ++pos;
          }
          return pos == line.size() ? (*content_length = value, true) : false;
        }
        if (!std::isdigit(ch))
        {
          return false;
        }
        const std::size_t digit = static_cast<std::size_t>(ch - '0');
        if (value > (std::numeric_limits<std::size_t>::max() - digit) / 10)
        {
          return false;
        }
        value = value * 10 + digit;
      }
      *content_length = value;
      return true;
    }
    line_begin = line_end + 2;
  }
  return false;
}

}  // namespace

namespace internal
{

HttpParseStatus ExtractHttpBody(const std::string& response, bool eof, std::string* body)
{
  std::size_t header_end = response.find("\r\n\r\n");
  if (header_end == std::string::npos)
  {
    return eof ? HttpParseStatus::kInvalid : HttpParseStatus::kIncomplete;
  }

  const std::string headers = response.substr(0, header_end + 2);
  std::size_t status_end = headers.find("\r\n");
  if (status_end == std::string::npos ||
      (headers.compare(0, 12, "HTTP/1.1 200") != 0 &&
       headers.compare(0, 12, "HTTP/1.0 200") != 0))
  {
    return HttpParseStatus::kInvalid;
  }

  constexpr std::size_t kMaxBodyBytes = 128 * 1024;
  const std::size_t body_begin = header_end + 4;
  std::size_t content_length = 0;
  if (ParseContentLength(headers, &content_length))
  {
    if (content_length > kMaxBodyBytes)
    {
      return HttpParseStatus::kInvalid;
    }
    if (response.size() - body_begin < content_length)
    {
      return eof ? HttpParseStatus::kInvalid : HttpParseStatus::kIncomplete;
    }
    *body = response.substr(body_begin, content_length);
    return HttpParseStatus::kComplete;
  }

  if (!eof)
  {
    return HttpParseStatus::kIncomplete;
  }
  if (response.size() - body_begin > kMaxBodyBytes)
  {
    return HttpParseStatus::kInvalid;
  }
  *body = response.substr(body_begin);
  return HttpParseStatus::kComplete;
}

}  // namespace internal

ReferenceFrameClient::ReferenceFrameClient(ReferenceFrameClientConfig config, ReferenceFrameCache* cache)
    : config_(std::move(config)), cache_(cache)
{
  if (!ParseHttpUrl(config_.url, &host_, &port_, &path_))
  {
    url_valid_ = false;
    std::cerr << "Invalid ghost reference URL: " << config_.url << std::endl;
  }
}

ReferenceFrameClient::~ReferenceFrameClient()
{
  Stop();
}

void ReferenceFrameClient::Start()
{
  if (thread_.joinable())
  {
    return;
  }
  stop_.store(false);
  thread_ = std::thread(&ReferenceFrameClient::Run, this);
}

void ReferenceFrameClient::Stop()
{
  stop_.store(true);
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void ReferenceFrameClient::Run()
{
  const double hz = std::max(1.0, config_.poll_hz);
  const auto poll_interval = std::chrono::duration<double>(1.0 / hz);
  int consecutive_failures = 0;

  while (!stop_.load())
  {
    auto start = std::chrono::steady_clock::now();
    std::string body;
    if (FetchOnce(&body))
    {
      ParseResult parsed = ParseReferenceFrameJson(body);
      if (parsed.ok)
      {
        consecutive_failures = 0;
        if (parsed.active)
        {
          cache_->StoreFrame(parsed.frame);
        }
        else
        {
          cache_->StoreInactive();
        }
      }
      else
      {
        ++consecutive_failures;
      }
    }
    else
    {
      ++consecutive_failures;
    }

    auto delay = poll_interval - (std::chrono::steady_clock::now() - start);
    if (consecutive_failures > 0)
    {
      const auto backoff = std::chrono::duration<double>(
          std::chrono::milliseconds(std::min(500, 50 * consecutive_failures)));
      if (backoff > delay)
      {
        delay = backoff;
      }
    }
    if (delay > std::chrono::milliseconds(0))
    {
      std::this_thread::sleep_for(delay);
    }
  }
}

bool ReferenceFrameClient::FetchOnce(std::string* body)
{
  if (!url_valid_)
  {
    return false;
  }

  int sock = ConnectWithTimeout(host_, port_, config_.timeout_ms);
  if (sock < 0)
  {
    return false;
  }

  timeval tv{};
  tv.tv_sec = config_.timeout_ms / 1000;
  tv.tv_usec = (config_.timeout_ms % 1000) * 1000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  std::string request =
      "GET " + path_ + " HTTP/1.1\r\n"
      "Host: " + host_ + "\r\n"
      "Connection: close\r\n"
      "Accept: application/json\r\n\r\n";

  if (!SendAll(sock, request))
  {
    close(sock);
    return false;
  }

  std::string response;
  char buffer[2048];
  while (response.size() < 128 * 1024)
  {
    ssize_t n = recv(sock, buffer, sizeof(buffer), 0);
    if (n > 0)
    {
      response.append(buffer, buffer + n);
      auto status = internal::ExtractHttpBody(response, false, body);
      if (status == internal::HttpParseStatus::kComplete)
      {
        close(sock);
        return true;
      }
      if (status == internal::HttpParseStatus::kInvalid)
      {
        close(sock);
        return false;
      }
      continue;
    }
    if (n == 0)
    {
      break;
    }
    if (errno == EINTR)
    {
      continue;
    }
    close(sock);
    return false;
  }

  auto status = internal::ExtractHttpBody(response, true, body);
  close(sock);
  return status == internal::HttpParseStatus::kComplete;
}

}  // namespace agentic_ref
