#include "reference_frame_cache.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace agentic_ref
{
namespace
{

void SkipWs(const std::string& s, std::size_t* pos)
{
  while (*pos < s.size() && std::isspace(static_cast<unsigned char>(s[*pos])))
  {
    ++(*pos);
  }
}

bool Consume(const std::string& s, std::size_t* pos, char expected)
{
  SkipWs(s, pos);
  if (*pos >= s.size() || s[*pos] != expected)
  {
    return false;
  }
  ++(*pos);
  return true;
}

std::optional<std::size_t> FindKeyValue(const std::string& s, const char* key)
{
  const std::string needle = "\"" + std::string(key) + "\"";
  std::size_t key_pos = s.find(needle);
  if (key_pos == std::string::npos)
  {
    return std::nullopt;
  }
  std::size_t pos = key_pos + needle.size();
  SkipWs(s, &pos);
  if (pos >= s.size() || s[pos] != ':')
  {
    return std::nullopt;
  }
  ++pos;
  SkipWs(s, &pos);
  return pos;
}

bool ParseBoolAt(const std::string& s, std::size_t pos, bool* value)
{
  if (s.compare(pos, 4, "true") == 0)
  {
    *value = true;
    return true;
  }
  if (s.compare(pos, 5, "false") == 0)
  {
    *value = false;
    return true;
  }
  return false;
}

bool ParseBoolKey(const std::string& s, const char* key, bool* value)
{
  auto pos = FindKeyValue(s, key);
  return pos && ParseBoolAt(s, *pos, value);
}

bool ParseStringAt(const std::string& s, std::size_t* pos, std::string* out)
{
  SkipWs(s, pos);
  if (*pos >= s.size() || s[*pos] != '"')
  {
    return false;
  }
  ++(*pos);
  out->clear();
  while (*pos < s.size())
  {
    char ch = s[(*pos)++];
    if (ch == '"')
    {
      return true;
    }
    if (ch == '\\')
    {
      if (*pos >= s.size())
      {
        return false;
      }
      char esc = s[(*pos)++];
      switch (esc)
      {
      case '"':
      case '\\':
      case '/':
        out->push_back(esc);
        break;
      case 'b':
        out->push_back('\b');
        break;
      case 'f':
        out->push_back('\f');
        break;
      case 'n':
        out->push_back('\n');
        break;
      case 'r':
        out->push_back('\r');
        break;
      case 't':
        out->push_back('\t');
        break;
      default:
        return false;
      }
    }
    else
    {
      out->push_back(ch);
    }
  }
  return false;
}

bool ParseStringKey(const std::string& s, const char* key, std::string* out)
{
  auto pos = FindKeyValue(s, key);
  if (!pos)
  {
    return false;
  }
  std::size_t p = *pos;
  return ParseStringAt(s, &p, out);
}

bool ParseNumberAt(const std::string& s, std::size_t* pos, double* out)
{
  SkipWs(s, pos);
  if (*pos >= s.size())
  {
    return false;
  }
  const char* begin = s.c_str() + *pos;
  char* end = nullptr;
  *out = std::strtod(begin, &end);
  if (end == begin)
  {
    return false;
  }
  if (!std::isfinite(*out))
  {
    return false;
  }
  *pos = static_cast<std::size_t>(end - s.c_str());
  return true;
}

bool ParseNumberKey(const std::string& s, const char* key, double* out)
{
  auto pos = FindKeyValue(s, key);
  if (!pos)
  {
    return false;
  }
  std::size_t p = *pos;
  return ParseNumberAt(s, &p, out);
}

bool ParseVec3(const std::string& s, std::size_t* pos, std::array<double, 3>* out)
{
  if (!Consume(s, pos, '['))
  {
    return false;
  }
  for (int i = 0; i < 3; ++i)
  {
    if (!ParseNumberAt(s, pos, &(*out)[i]))
    {
      return false;
    }
    if (i < 2 && !Consume(s, pos, ','))
    {
      return false;
    }
  }
  return Consume(s, pos, ']');
}

bool ParseVec4(const std::string& s, std::size_t* pos, std::array<double, 4>* out)
{
  if (!Consume(s, pos, '['))
  {
    return false;
  }
  for (int i = 0; i < 4; ++i)
  {
    if (!ParseNumberAt(s, pos, &(*out)[i]))
    {
      return false;
    }
    if (i < 3 && !Consume(s, pos, ','))
    {
      return false;
    }
  }
  return Consume(s, pos, ']');
}

bool ParseVec3Array27(const std::string& s, const char* key,
                      std::array<std::array<double, 3>, kBodyCount>* out)
{
  auto pos_opt = FindKeyValue(s, key);
  if (!pos_opt)
  {
    return false;
  }
  std::size_t pos = *pos_opt;
  if (!Consume(s, &pos, '['))
  {
    return false;
  }
  for (int i = 0; i < kBodyCount; ++i)
  {
    if (!ParseVec3(s, &pos, &(*out)[i]))
    {
      return false;
    }
    if (i < kBodyCount - 1 && !Consume(s, &pos, ','))
    {
      return false;
    }
  }
  return Consume(s, &pos, ']');
}

bool ParseVec4Array27(const std::string& s, const char* key,
                      std::array<std::array<double, 4>, kBodyCount>* out)
{
  auto pos_opt = FindKeyValue(s, key);
  if (!pos_opt)
  {
    return false;
  }
  std::size_t pos = *pos_opt;
  if (!Consume(s, &pos, '['))
  {
    return false;
  }
  for (int i = 0; i < kBodyCount; ++i)
  {
    if (!ParseVec4(s, &pos, &(*out)[i]))
    {
      return false;
    }
    if (i < kBodyCount - 1 && !Consume(s, &pos, ','))
    {
      return false;
    }
  }
  return Consume(s, &pos, ']');
}

bool ParseContacts(const std::string& s, std::array<bool, 2>* out)
{
  auto pos_opt = FindKeyValue(s, "c");
  if (!pos_opt)
  {
    return false;
  }
  std::size_t pos = *pos_opt;
  if (!Consume(s, &pos, '['))
  {
    return false;
  }
  for (int i = 0; i < 2; ++i)
  {
    SkipWs(s, &pos);
    bool b = false;
    if (ParseBoolAt(s, pos, &b))
    {
      out->at(i) = b;
      pos += b ? 4 : 5;
    }
    else
    {
      double value = 0.0;
      if (!ParseNumberAt(s, &pos, &value))
      {
        return false;
      }
      out->at(i) = value != 0.0;
    }
    if (i < 1 && !Consume(s, &pos, ','))
    {
      return false;
    }
  }
  return Consume(s, &pos, ']');
}

bool ParseRequiredNumber(const std::string& s, const char* key, double* out, std::string* error)
{
  if (!ParseNumberKey(s, key, out))
  {
    *error = std::string("missing or invalid ") + key;
    return false;
  }
  return true;
}

bool ParseRequiredInt(const std::string& s, const char* key, int* out, std::string* error)
{
  double value = 0.0;
  if (!ParseRequiredNumber(s, key, &value, error))
  {
    return false;
  }
  if (value < static_cast<double>(std::numeric_limits<int>::min()) ||
      value > static_cast<double>(std::numeric_limits<int>::max()))
  {
    *error = std::string("missing or invalid ") + key;
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

}  // namespace

ParseResult ParseReferenceFrameJson(const std::string& json)
{
  ParseResult result;

  bool ok = false;
  if (!ParseBoolKey(json, "ok", &ok) || !ok)
  {
    result.error = "ok is not true";
    return result;
  }

  bool active = false;
  if (!ParseBoolKey(json, "active", &active))
  {
    result.error = "missing active";
    return result;
  }
  result.active = active;
  if (!active)
  {
    result.ok = true;
    return result;
  }

  ReferenceFrame frame;
  frame.active = true;
  frame.received_at = std::chrono::steady_clock::now();

  if (!ParseStringKey(json, "schema", &frame.schema))
  {
    result.error = "missing schema";
    return result;
  }
  if (frame.schema != kSchema)
  {
    result.error = "unsupported schema";
    return result;
  }
  if (!ParseStringKey(json, "body_order", &frame.body_order))
  {
    result.error = "missing body_order";
    return result;
  }
  if (frame.body_order != kBodyOrder)
  {
    result.error = "unsupported body_order";
    return result;
  }
  ParseStringKey(json, "id", &frame.id);

  if (!ParseRequiredInt(json, "frame", &frame.frame, &result.error)) return result;
  if (!ParseRequiredInt(json, "frames", &frame.frames, &result.error)) return result;
  if (!ParseRequiredNumber(json, "time_s", &frame.time_s, &result.error)) return result;
  if (!ParseRequiredNumber(json, "fps", &frame.fps, &result.error)) return result;
  if (!ParseRequiredNumber(json, "stale_ms", &frame.stale_ms, &result.error)) return result;

  if (!ParseVec3Array27(json, "p", &frame.p))
  {
    result.error = "missing or invalid p";
    return result;
  }
  if (!ParseVec4Array27(json, "q", &frame.q))
  {
    result.error = "missing or invalid q";
    return result;
  }
  if (!ParseContacts(json, &frame.c))
  {
    result.error = "missing or invalid c";
    return result;
  }

  auto com_pos = FindKeyValue(json, "com");
  if (!com_pos)
  {
    result.error = "missing com";
    return result;
  }
  std::size_t pos = *com_pos;
  if (!ParseVec3(json, &pos, &frame.com))
  {
    result.error = "invalid com";
    return result;
  }

  auto comv_pos = FindKeyValue(json, "comv");
  if (!comv_pos)
  {
    result.error = "missing comv";
    return result;
  }
  pos = *comv_pos;
  if (!ParseVec3(json, &pos, &frame.comv))
  {
    result.error = "invalid comv";
    return result;
  }

  result.ok = true;
  result.frame = frame;
  return result;
}

void ReferenceFrameCache::StoreInactive()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = ReferenceFrame{};
  latest_.active = false;
  latest_.received_at = std::chrono::steady_clock::now();
  has_latest_ = true;
}

void ReferenceFrameCache::StoreFrame(const ReferenceFrame& frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = frame;
  latest_.received_at = std::chrono::steady_clock::now();
  has_latest_ = true;
}

std::optional<ReferenceFrame> ReferenceFrameCache::LatestFresh(
    std::chrono::milliseconds max_local_age,
    double max_packet_stale_ms) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_latest_ || !latest_.active)
  {
    return std::nullopt;
  }
  if (latest_.schema != kSchema || latest_.body_order != kBodyOrder)
  {
    return std::nullopt;
  }
  if (latest_.stale_ms > max_packet_stale_ms)
  {
    return std::nullopt;
  }
  auto age = std::chrono::steady_clock::now() - latest_.received_at;
  if (age > max_local_age)
  {
    return std::nullopt;
  }
  return latest_;
}

}  // namespace agentic_ref
