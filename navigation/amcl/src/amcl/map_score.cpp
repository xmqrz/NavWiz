#include <amcl/map_score.h>

#include <boost/filesystem.hpp>
#include <ctime>
#include <png++/png.hpp>
#include <tf2/utils.h>


/* Internal implementation */

namespace fs = boost::filesystem;

const char* FILENAME_FORMAT = "%Y%m%d_%H%M%S";
const int FILENAME_LENGTH = 15;
const char* FILENAME_DATE_FORMAT = "%Y%m%d";
const int FILENAME_DATE_LENGTH = 8;

struct QualityMap
{
  // constructor: load data from file storage
  explicit QualityMap(const std::string& base_filename);

  // constructor: create from OccupancyGrid message
  explicit QualityMap(const nav_msgs::OccupancyGrid& msg, const std::string& map_layout);

  // copy constructor: rotate png files and discard score data
  explicit QualityMap(const QualityMap& qmap);

  // check if msg is the reference map
  inline bool isReferenceMap(const nav_msgs::OccupancyGrid& msg) const;
  inline bool isReferenceMap(const QualityMap& qmap) const;

  // convert from world coords to grid coords
  inline bool worldToGrid(double wx, double wy, int& mx, int& my) const;

  // update map origin, resolution and map layout
  void update(const nav_msgs::OccupancyGrid& msg, const std::string& map_layout);
  void update(const QualityMap& qmap);

  // write quality scores to png files
  void write();

  // generate filename by using current time
  static std::string generateFilename();

  // generate color palette and transparency
  static void setPalette();
  static void setPalette(double high, double low);
  static const png::palette& getPalette();
  static const png::tRNS& getTRNS();

  // variables
  ros::Time stamp;       // msg header timestamp
  uint32_t crc;          // png IDAT chunk CRC
  uint32_t width;        // map width
  uint32_t height;       // map height
  double resolution;     // map scale (m/cell)
  double origin_x;       // map origin in world coordinates
  double origin_y;       //
  std::string filename;  // base filename
  png::image<png::index_pixel> min;   // minimum quality score
  png::image<png::index_pixel> max;   // maximum quality score
  png::image<png::index_pixel> avg;   // average quality score
  png::image<png::rgba_pixel> sum;    // total quality score
  png::image<png::rgba_pixel> count;  // number of observations
  bool dirty;

  static std::string folder;  // storage folder

  static png::palette palette;  // color palette
  static double high_threshold;
  static double low_threshold;
};

// utility functions
template<size_t size> uint8_t basisSpline(uint8_t (&values)[size], float t);
inline std::string getFilename(const time_t& t);
inline std::string getFilenameDate(const time_t& t);
uint32_t extractCrc(const std::vector<int8_t>& data);

// stamp and crc are serialized and deserialized from the PNG palette
BOOST_STATIC_ASSERT(offsetof(QualityMap, stamp) == 0);
BOOST_STATIC_ASSERT(sizeof(((QualityMap *) 0)->stamp) == 8);
BOOST_STATIC_ASSERT(offsetof(QualityMap, crc) == 8);
BOOST_STATIC_ASSERT(sizeof(((QualityMap *) 0)->crc) == 4);

// class static variables
std::string QualityMap::folder;
png::palette QualityMap::palette(256);
double QualityMap::high_threshold = -1.0;
double QualityMap::low_threshold = -1.0;

typedef png::require_color_space<png::index_pixel> require_index_pixel;
typedef png::require_color_space<png::rgba_pixel> require_rgba_pixel;


// pixel_generator class to write OCG to png file
class pixel_generator: public png::generator<png::index_pixel, pixel_generator>
{
public:
  pixel_generator(size_t width, size_t height, const std::vector<int8_t>& pixels) :
    png::generator<png::index_pixel, pixel_generator>(width, height),
    pixels_(pixels)
  {
    // palette
    png::palette& palette = get_info().get_palette();
    palette.clear();
    palette.resize(256);
    palette[0] = png::color(255, 255, 255);
    for (int i = 1; i < 100; ++i)
    {
      int g = 200 - i;
      palette[i].red = g;
      palette[i].green = g;
      palette[i].blue = g;
    }

    // transparency
    png::tRNS& trns = get_info().get_tRNS();
    trns.clear();
    trns.resize(256);
    for (int i = 0; i < 101; ++i)
    {
      trns[i] = 255;
    }
    for (int i = 101; i < 256; ++i)
    {
      trns[i] = 0;
    }
  }
  png::byte* get_next_row(size_t pos)
  {
    return reinterpret_cast<png::byte*>(const_cast<int8_t*>(&pixels_[pos * get_info().get_width()]));
  }
protected:
  const std::vector<int8_t>& pixels_;
};


// constructor: load data from file storage
QualityMap::QualityMap(const std::string& base_filename) :
  filename(base_filename),
  min(folder + filename + "_min.png", require_index_pixel()),
  max(folder + filename + "_max.png", require_index_pixel()),
  avg(folder + filename + "_avg.png", require_index_pixel()),
  sum(folder + filename + "_sum.png", require_rgba_pixel()),
  count(folder + filename + "_count.png", require_rgba_pixel()),
  dirty(false)
{
  width = min.get_width();
  height = min.get_height();

  if (max.get_width() != width || max.get_height() != height ||
      avg.get_width() != width || avg.get_height() != height ||
      sum.get_width() != width || sum.get_height() != height ||
      count.get_width() != width || count.get_height() != height)
  {
    throw std::runtime_error("Dimension mismatch");
  }

  // retrieve stamp and crc from palette entries 251-254.
  memcpy(this, &avg.get_palette()[251], 12);
}

// constructor: create from OccupancyGrid message
QualityMap::QualityMap(const nav_msgs::OccupancyGrid& msg, const std::string& map_layout) :
  stamp(msg.header.stamp),
  crc(msg.info.map_load_time.sec),  // CRC must be extracted from IDAT chunk and stored in map_load_time.sec
  width(msg.info.width), height(msg.info.height),
  resolution(msg.info.resolution),
  origin_x(msg.info.origin.position.x), origin_y(msg.info.origin.position.y),
  filename(generateFilename()),
  min(width, height),
  max(width, height),
  avg(width, height),
  sum(width, height),
  count(width, height),
  dirty(false)
{
  // write map_layout, ocg_metadata and OCG
  try
  {
    std::ofstream ofs((folder + filename + "_layout.json").c_str());
    ofs << map_layout << std::endl;
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error writing map layout file: " << ex.what());
  }
  try
  {
    std::ofstream ofs((folder + filename + "_ocg.json").c_str());
    ofs << "{\"width\":" << width;
    ofs << ",\"height\":" << height;
    ofs << ",\"resolution\":" << resolution;
    ofs << ",\"x0\":" << origin_x;
    ofs << ",\"y0\":" << origin_y;
    ofs << "}" << std::endl;
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error writing ocg metadata file: " << ex.what());
  }
  try
  {
    std::ofstream ofs((folder + filename + "_ocg.png").c_str());
    if (crc)
    {
      ofs.write(reinterpret_cast<const char*>(msg.data.data()), msg.data.size());
    }
    else
    {
      pixel_generator(width, height, msg.data).write(ofs);
    }
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error writing ocg png file: " << ex.what());
  }

  // set palette and tRNS chunks
  min.set_palette(getPalette());
  max.set_palette(getPalette());
  avg.set_palette(getPalette());
  min.set_tRNS(getTRNS());
  max.set_tRNS(getTRNS());
  avg.set_tRNS(getTRNS());

  // store stamp and crc in palette entries 251-254.
  memcpy(&avg.get_palette()[251], this, 12);

  // reset image data to 255 (unknown)
  for (int i = 0; i < height; ++i)
  {
    std::fill(min[i].begin(), min[i].end(), 255);
    std::fill(max[i].begin(), max[i].end(), 255);
    std::fill(avg[i].begin(), avg[i].end(), 255);
  }
}

// copy constructor: rotate png files and discard score data
QualityMap::QualityMap(const QualityMap& qmap) :
  stamp(qmap.stamp), crc(qmap.crc),
  width(qmap.width), height(qmap.height),
  resolution(qmap.resolution),
  origin_x(qmap.origin_x), origin_y(qmap.origin_y),
  filename(generateFilename()),
  min(width, height),
  max(width, height),
  avg(width, height),
  sum(width, height),
  count(width, height),
  dirty(false)
{
  // copy map_layout, ocg_metadata and OCG
  try
  {
    fs::copy_file(fs::path(folder + qmap.filename + "_layout.json"),
                  fs::path(folder + filename + "_layout.json"));
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error copying map layout file: " << ex.what());
  }
  try
  {
    fs::copy_file(fs::path(folder + qmap.filename + "_ocg.json"),
                  fs::path(folder + filename + "_ocg.json"));
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error copying ocg metadata file: " << ex.what());
  }
  try
  {
    fs::copy_file(fs::path(folder + qmap.filename + "_ocg.png"),
                  fs::path(folder + filename + "_ocg.png"));
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error copying ocg png file: " << ex.what());
  }

  // set palette and tRNS chunks
  min.set_palette(getPalette());
  max.set_palette(getPalette());
  avg.set_palette(getPalette());
  min.set_tRNS(getTRNS());
  max.set_tRNS(getTRNS());
  avg.set_tRNS(getTRNS());

  // store stamp and crc in palette entries 251-254.
  memcpy(&avg.get_palette()[251], this, 12);

  // reset image data to 255 (unknown)
  for (int i = 0; i < height; ++i)
  {
    std::fill(min[i].begin(), min[i].end(), 255);
    std::fill(max[i].begin(), max[i].end(), 255);
    std::fill(avg[i].begin(), avg[i].end(), 255);
  }
}

inline bool QualityMap::isReferenceMap(const nav_msgs::OccupancyGrid& msg) const
{
  if (msg.info.width != width || msg.info.height != height)
  {
    return false;
  }
  if (msg.info.map_load_time.sec == 0 || crc == 0)  // msg's CRC or previous CRC record empty
  {
    return msg.header.stamp == stamp;
  }
  return msg.info.map_load_time.sec == crc;
}

inline bool QualityMap::isReferenceMap(const QualityMap& qmap) const
{
  if (qmap.width != width || qmap.height != height)
  {
    return false;
  }
  if (qmap.crc == 0 || crc == 0)
  {
    return qmap.stamp == stamp;
  }
  return qmap.crc == crc;
}

inline bool QualityMap::worldToGrid(double wx, double wy, int& mx, int& my) const
{
  mx = std::floor((wx - origin_x) / resolution + 0.5);
  my = std::floor((wy - origin_y) / resolution + 0.5);
  return mx >= 0 && mx < width && my >= 0 && my < height;
}

void QualityMap::update(const nav_msgs::OccupancyGrid& msg, const std::string& map_layout)
{
  resolution = msg.info.resolution;
  origin_x = msg.info.origin.position.x;
  origin_y = msg.info.origin.position.y;

  if (stamp != msg.header.stamp)
  {
    stamp = msg.header.stamp;

    // overwrite map_layout and ocg_metadata
    try
    {
      std::ofstream ofs((folder + filename + "_layout.json").c_str());
      ofs << map_layout << std::endl;
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("Error writing map layout file: " << ex.what());
    }
    try
    {
      std::ofstream ofs((folder + filename + "_ocg.json").c_str());
      ofs << "{\"width\":" << width;
      ofs << ",\"height\":" << height;
      ofs << ",\"resolution\":" << resolution;
      ofs << ",\"x0\":" << origin_x;
      ofs << ",\"y0\":" << origin_y;
      ofs << "}" << std::endl;
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("Error writing ocg metadata file: " << ex.what());
    }
  }

  if (msg.info.map_load_time.sec != 0)
  {
    crc = msg.info.map_load_time.sec;
  }

  // store stamp and crc in palette entries 251-254.
  memcpy(&avg.get_palette()[251], this, 12);
}

void QualityMap::update(const QualityMap& qmap)
{
  resolution = qmap.resolution;
  origin_x = qmap.origin_x;
  origin_y = qmap.origin_y;

  if (stamp != qmap.stamp)
  {
    stamp = qmap.stamp;

    // overwrite map_layout and ocg_metadata
    try
    {
      fs::copy_file(fs::path(folder + qmap.filename + "_layout.json"),
                    fs::path(folder + filename + "_layout.json"),
                    fs::copy_option::overwrite_if_exists);
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("Error copying map layout file: " << ex.what());
    }
    try
    {
      fs::copy_file(fs::path(folder + qmap.filename + "_ocg.json"),
                    fs::path(folder + filename + "_ocg.json"),
                    fs::copy_option::overwrite_if_exists);
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("Error copying ocg metadata file: " << ex.what());
    }
  }

  if (qmap.crc != 0)
  {
    crc = qmap.crc;
  }

  // store stamp and crc in palette entries 251-254.
  memcpy(&avg.get_palette()[251], this, 12);
}

void QualityMap::write()
{
  try
  {
    min.write(folder + filename + "_min.png");
    max.write(folder + filename + "_max.png");
    avg.write(folder + filename + "_avg.png");
    sum.write(folder + filename + "_sum.png");
    count.write(folder + filename + "_count.png");
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error writing quality map png file: " << ex.what());
  }
}

std::string QualityMap::generateFilename()
{
  time_t now = time(NULL);
  std::string filename = getFilename(now);

  // ensure it is unique
  while (fs::exists(fs::path(folder + filename + "_layout.json")) ||
         fs::exists(fs::path(folder + filename + "_ocg.json")) ||
         fs::exists(fs::path(folder + filename + "_ocg.png")) ||
         fs::exists(fs::path(folder + filename + "_min.png")) ||
         fs::exists(fs::path(folder + filename + "_max.png")) ||
         fs::exists(fs::path(folder + filename + "_avg.png")) ||
         fs::exists(fs::path(folder + filename + "_sum.png")) ||
         fs::exists(fs::path(folder + filename + "_count.png")))
  {
    now += 1;
    filename = getFilename(now);
  }
  return filename;
}

void QualityMap::setPalette()
{
  // Adapted from https://github.com/d3/d3-scale-chromatic/blob/main/src/diverging/RdYlGn.js
  // and https://github.com/d3/d3-interpolate/blob/main/src/rgb.js

  // Color Scheme: Red - Yellow - Green (0 - 100%)
  uint32_t scheme[] =
  {
    0xa50026, 0xd73027, 0xf46d43, 0xfdae61, 0xfee08b, 0xffffbf,
    0xd9ef8b, 0xa6d96a, 0x66bd63, 0x1a9850, 0x006837
  };

  const int n = sizeof(scheme) / sizeof(scheme[0]);
  uint8_t r[n];
  uint8_t g[n];
  uint8_t b[n];

  for (int i = 0; i < n; ++i)
  {
    uint32_t v = scheme[i];
    b[i] = v & 0xff;
    g[i] = (v >>= 8) & 0xff;
    r[i] = (v >>= 8) & 0xff;
  }

  // Interpolate each RGB component using the B-spline function
  for (int i = 0; i < 101; ++i)
  {
    float t = i * 0.01;
    palette[i].red = basisSpline(r, t);
    palette[i].green = basisSpline(g, t);
    palette[i].blue = basisSpline(b, t);
  }
}

void QualityMap::setPalette(double high, double low)
{
  high = std::min(std::max(high, 0.0), 100.0);
  low = std::min(std::max(low, 0.0), high);

  if (high_threshold == high && low_threshold == low)
  {
    return;
  }
  high_threshold = high;
  low_threshold = low;

  if (high > 0.0)
  {
    int i = 0;
    while (static_cast<double>(i) < low)
    {
      // Crimson
      palette[i].red = 0xdc;
      palette[i].green = 0x14;
      palette[i].blue = 0x3c;
      ++i;
    }
    while (static_cast<double>(i) < high)
    {
      // Gold
      palette[i].red = 0xff;
      palette[i].green = 0xd7;
      palette[i].blue = 0x00;
      ++i;
    }
    while (i < 101)
    {
      // MediumSeaGreen
      palette[i].red = 0x3c;
      palette[i].green = 0xb3;
      palette[i].blue = 0x71;
      ++i;
    }
  }
  else
  {
    setPalette();
  }
}

const png::palette& QualityMap::getPalette()
{
  if (high_threshold < 0.0)
  {
    setPalette(0.0, 0.0);
  }
  return palette;
}

const png::tRNS& QualityMap::getTRNS()
{
  static png::tRNS trns(256);
  static bool generated = false;
  if (generated)
  {
    return trns;
  }

  // transparency
  for (int i = 0; i < 101; ++i)
  {
    trns[i] = 255;
  }
  for (int i = 101; i < 256; ++i)
  {
    trns[i] = 0;
  }
  generated = true;
  return trns;
}

template<size_t size>
uint8_t basisSpline(uint8_t (&values)[size], float t)
{
  // Adapted from https://github.com/d3/d3-interpolate/blob/main/src/basis.js
  int n = size - 1;
  int i = (t <= 0) ? (t = 0) : (t >= 1) ? (t = 1, n - 1) : t * n;

  float v1 = values[i];
  float v2 = values[i + 1];
  float v0 = i > 0 ? values[i - 1] : 2 * v1 - v2;
  float v3 = i < n - 1 ? values[i + 2] : 2 * v2 - v1;

  t = t * n - i;
  float t2 = t * t;
  float t3 = t2 * t;

  return ((1 - 3 * t + 3 * t2 - t3) * v0 +
          (4 - 6 * t2 + 3 * t3) * v1 +
          (1 + 3 * t + 3 * t2 - 3 * t3) * v2 +
          t3 * v3) / 6;
}

inline std::string getFilename(const time_t& t)
{
  std::string s;
  s.resize(FILENAME_LENGTH);
  strftime(&s[0], FILENAME_LENGTH + 1, FILENAME_FORMAT, localtime(&t));
  return s;
}

inline std::string getFilenameDate(const time_t& t)
{
  std::string s;
  s.resize(FILENAME_DATE_LENGTH);
  strftime(&s[0], FILENAME_DATE_LENGTH + 1, FILENAME_DATE_FORMAT, localtime(&t));
  return s;
}

// extract CRC from png IDAT chunk
uint32_t extractCrc(const std::vector<int8_t>& data)
{
  const int8_t PNG_MAGIC_BYTES[8] = {-0x77, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  if (data.size() <= 8 ||
      !std::equal(data.begin(), data.begin() + 8, PNG_MAGIC_BYTES))
  {
    return 0;
  }

  const int8_t* p = data.data() + 8;
  const int8_t* end = data.data() + data.size();

  while (p < end)
  {
    uint32_t len = __builtin_bswap32(*reinterpret_cast<const uint32_t*>(p));
    p += 4;  // skip length (4 bytes)

    if (memcmp(p, "IDAT", 4) == 0)
    {
      p += 4 + len;  // skip chunk type (4 bytes) and data (len bytes)

      uint32_t crc = __builtin_bswap32(*reinterpret_cast<const uint32_t*>(p));
      if (crc == 0)
      {
        crc = 1;  // avoid zero CRC, bump to 1
      }
      return crc;
    }
    p += 4 + len + 4;  // skip chunk type (4 bytes), data (len bytes) and CRC (4 bytes)
  }
  return 0;
}


/* MapScore class */

void MapScore::reconfigure(amcl::AMCLConfig& config)
{
  if (config.low_threshold > config.high_threshold)
  {
    config.low_threshold = config.high_threshold;
  }
  QualityMap::setPalette(config.high_threshold * 100.0, config.low_threshold * 100.0);

  if (config.new_map_score_)
  {
    config.new_map_score_ = false;
    new_qmap_ = true;

    if (!current_qmap_)
    {
      boost::shared_ptr<nav_msgs::OccupancyGrid const> msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
      if (msg)
      {
        updateMap(*msg, map_layout_);
      }
    }

    try
    {
      std::ofstream ofs((QualityMap::folder + "new_map_score").c_str());
      ofs << QualityMap::generateFilename() << std::endl;
    }
    catch (const std::exception& ex)
    {
      ROS_WARN_STREAM("Error writing new_map_score: " << ex.what());
    }
    cv_.notify_one();
  }
}

void MapScore::loadQmaps()
{
  // create folder for data storage
  std::string folder;
  const char* home = getenv("HOME");
  if (home == NULL)
  {
    ROS_WARN("HOME environment variable is not set. Falling back to /tmp folder.");
    folder = "/tmp";
  }
  else
  {
    folder = home;
  }
  folder += "/.ros/map_quality/";

  fs::create_directories(folder);
  QualityMap::folder = folder;

  // generate today's and last month's date
  time_t now = time(NULL);
  today_ = getFilenameDate(now);
  std::string last_month = getFilenameDate(now - 30 * 24 * 3600);

  try
  {
    std::string filename = QualityMap::folder + "new_map_score";
    std::ifstream ifs(filename.c_str());
    if (ifs.is_open())
    {
      std::string today;
      std::getline(ifs, today);
      ifs.close();

      if (today.compare(0, today_.size(), today_))
      {
        // new_map_score not today
        fs::remove(filename);
      }
      else
      {
        today_ = today;
      }
    }
  }
  catch (const std::exception& ex)
  {
    ROS_WARN_STREAM("Error reading new_map_score: " << ex.what());
  }

  // read existing quality maps
  qmaps_.clear();
  for (fs::directory_iterator it(folder), it_end; it != it_end; ++it)
  {
    std::string filename = it->path().filename().string();

    if (filename.compare(0, last_month.size(), last_month) < 0)
    {
      // remove files older than last month
      try
      {
        fs::remove(it->path());
      }
      catch (const std::exception& ex)
      {}
      continue;
    }

    if (!current_qmap_)
    {
      // map quality score logging is off
      continue;
    }

    if (filename.compare(0, today_.size(), today_) < 0)
    {
      // filename not today latest
      continue;
    }

    if (filename.size() <= 8 ||
        filename.compare(filename.size() - 8, std::string::npos, "_avg.png") != 0)
    {
      // skip non-relevant files
      continue;
    }
    filename.resize(filename.size() - 8);

    struct tm t;
    if (!strptime(filename.c_str(), FILENAME_FORMAT, &t))
    {
      // invalid filename format
      continue;
    }

    boost::shared_ptr<QualityMap> qmap;
    try
    {
      qmap = boost::make_shared<QualityMap>(filename);
    }
    catch (const std::exception& ex)
    {
      continue;
    }
    qmaps_.push_back(qmap);
  }
}

void MapScore::updateMap(const nav_msgs::OccupancyGrid& msg, const std::string& map_layout)
{
  // mark inlier data invalid
  for (auto& key : inlier_perc_)
  {
    key.second.invalid_ = true;
  }

  // extract crc from png IDAT chunk
  const_cast<nav_msgs::OccupancyGrid&>(msg).info.map_load_time.sec = extractCrc(msg.data);

  // lookup existing qmaps
  Lock lock(mutex_);
  for (int i = 0, n = qmaps_.size(); i < n; ++i)
  {
    if (qmaps_[i]->isReferenceMap(msg))
    {
      qmaps_[i]->update(msg, map_layout);

      Lock lock_fast(mutex_fast_);
      current_qmap_ = qmaps_[i];
      return;
    }
  }

  // create new qmap if not found
  Lock lock_fast(mutex_fast_);
  current_qmap_ = boost::make_shared<QualityMap>(msg, map_layout);
  qmaps_.push_back(current_qmap_);
  new_qmap_ = false;
}

void MapScore::updateMap(const nav_msgs::OccupancyGrid& msg)
{
  if (current_qmap_)
  {
    updateMap(msg, map_layout_);
  }
}

void MapScore::update(const geometry_msgs::Pose& robot_pose,
                      const std::string& laser_scan_frame_id,
                      const pf_vector_t& laser_pose,
                      const amcl::AMCLLaserData& ldata,
                      InlierPercentage& inlier)
{
  pf_vector_t robot_pose_v;
  robot_pose_v.v[0] = robot_pose.position.x;
  robot_pose_v.v[1] = robot_pose.position.y;
  robot_pose_v.v[2] = tf2::getYaw(robot_pose.orientation);

  pf_vector_t laser_pose_in_map = pf_vector_coord_add(laser_pose, robot_pose_v);

  // compute inlier percentage
  InlierPercentage::Score score = inlier.computeInlierScore(ldata, laser_pose_in_map);

  if (score.invalid_)
  {
    return;
  }
  inlier_perc_[laser_scan_frame_id] = score;

  size_t count = 1;
  for (const auto& key : inlier_perc_)
  {
    if (key.first == laser_scan_frame_id)
    {
      continue;
    }
    if (key.second.invalid_)
    {
      continue;
    }
    if (score.landmarks_.size() != key.second.landmarks_.size())
    {
      continue;
    }
    ++count;
    score.inlier_ += key.second.inlier_;
    score.landmark_hit_ += key.second.landmark_hit_;
    score.landmark_found_ += key.second.landmark_found_;
    for (size_t i = 0; i < score.landmarks_.size(); ++i)
    {
      score.landmarks_[i] = std::max(score.landmarks_[i], key.second.landmarks_[i]);
    }
  }

  if (count > 1)
  {
    score.inlier_ /= count;
  }

  inlier_overall_ = inlier.computePercentage(score, inlier_landmark_) * 100.0;

  if (!current_qmap_ || !enable_)
  {
    return;
  }

  // update quality map
  boost::shared_ptr<QualityMap> qmap;
  {
    Lock lock_fast(mutex_fast_);
    qmap = current_qmap_;
  }
  ROS_ASSERT(qmap);

  int mx, my;
  if (!qmap->worldToGrid(robot_pose_v.v[0], robot_pose_v.v[1], mx, my))
  {
    return;
  }

  uint8_t v = inlier_overall_;
  uint8_t& min_v = reinterpret_cast<uint8_t&>(qmap->min[my][mx]);
  uint8_t& max_v = reinterpret_cast<uint8_t&>(qmap->max[my][mx]);
  uint8_t& avg_v = reinterpret_cast<uint8_t&>(qmap->avg[my][mx]);
  uint32_t& sum_v = reinterpret_cast<uint32_t&>(qmap->sum[my][mx]);
  uint32_t& count_v = reinterpret_cast<uint32_t&>(qmap->count[my][mx]);

  if (min_v == 255 || v < min_v)
  {
    min_v = v;
  }
  if (max_v == 255 || v > max_v)
  {
    max_v = v;
  }
  sum_v += v;
  count_v += 1;
  avg_v = sum_v / count_v;

  qmap->dirty = true;
}

void MapScore::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.addf("AMCL Inlier (%)", "%.2f", inlier_overall_);
  stat.addf("Landmark Inlier", "%.2f", inlier_landmark_);
  for (const auto& key : inlier_perc_)
  {
    stat.addf(key.first + " (%)", "%.2f", key.second.inlier_ * 100.0);
  }

  if (inlier_overall_ < QualityMap::low_threshold)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Too low");
  }
  else if (inlier_overall_ < QualityMap::high_threshold)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low");
  }
  else if (QualityMap::high_threshold > 0.0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "High");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
}

void MapScore::writerThread()
{
  time_t next_cycle = time(NULL) + 300;
  next_cycle -= next_cycle % 300;

  while (ros::ok() && !thread_shutdown_)
  {
    // loop once every 5 minutes
    Lock lock(mutex_);
    if (!thread_shutdown_)
    {
      cv_.timed_wait(lock, boost::posix_time::from_time_t(next_cycle));
    }
    next_cycle = time(NULL) + 300;
    next_cycle -= next_cycle % 300;

    // write to PNG files
    for (int i = 0, n = qmaps_.size(); i < n; ++i)
    {
      if (qmaps_[i]->dirty)
      {
        qmaps_[i]->dirty = false;
        qmaps_[i]->write();
      }
    }

    if (!current_qmap_ || !enable_)
    {
      continue;
    }

    // rotate qmaps daily
    std::string today = getFilenameDate(time(NULL));
    if (!today_.compare(0, today.size(), today) && !new_qmap_)
    {
      continue;
    }

    loadQmaps();

    boost::shared_ptr<QualityMap> qmap;
    {
      Lock lock_fast(mutex_fast_);
      qmap = current_qmap_;
    }
    if (!qmap)
    {
      continue;
    }

    if (!new_qmap_)
    {
      if (qmap->filename.compare(0, today.size(), today) == 0)
      {
        // qmap was newly created today in updateMap(), what a coincidence!
        qmaps_.push_back(qmap);
      }
      else
      {
        // lookup existing qmaps, just in case!
        bool found = false;
        for (int i = 0, n = qmaps_.size(); i < n; ++i)
        {
          if (qmaps_[i]->isReferenceMap(*qmap))
          {
            qmaps_[i]->update(*qmap);

            Lock lock_fast(mutex_fast_);
            current_qmap_ = qmaps_[i];
            found = true;
            break;
          }
        }

        if (!found)
        {
          new_qmap_ = true;
        }
      }
    }

    // rotate and create new qmap
    if (new_qmap_)
    {
      Lock lock_fast(mutex_fast_);
      current_qmap_ = boost::make_shared<QualityMap>(*current_qmap_);
      qmaps_.push_back(current_qmap_);
      new_qmap_ = false;
    }
  }
}
