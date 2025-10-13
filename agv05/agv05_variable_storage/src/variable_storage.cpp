/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <agv05_variable_storage/variable_storage.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <sstream>

#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>


namespace agv05
{

class VariableStorageBackend
{
public:
  virtual bool hasVariable(const std::string& name) = 0;
  virtual bool getVariable(const std::string& name, std::string& value) = 0;
  virtual void setVariable(const std::string& name, const std::string& value) = 0;
  virtual void deleteVariable(const std::string& name) = 0;
  static bool matchUrl(const std::string& storage_url);
};

class FileStorageBackend: public VariableStorageBackend
{
public:
  FileStorageBackend(const std::string& node_name, const std::string& storage_url);
  virtual bool hasVariable(const std::string& name);
  virtual bool getVariable(const std::string& name, std::string& value);
  virtual void setVariable(const std::string& name, const std::string& value);
  virtual void deleteVariable(const std::string& name);
  static bool matchUrl(const std::string& storage_url);

private:
  std::string folder_path_;
  bool initialized_;
  static const char* protocol_;
};

class MysqlStorageBackend: public VariableStorageBackend
{
public:
  MysqlStorageBackend(const std::string& node_name, const std::string& storage_url);
  virtual bool hasVariable(const std::string& name);
  virtual bool getVariable(const std::string& name, std::string& value);
  virtual void setVariable(const std::string& name, const std::string& value);
  virtual void deleteVariable(const std::string& name);
  static bool matchUrl(const std::string& storage_url);

private:
  std::string db_host_;
  std::string db_user_;
  std::string db_password_;
  std::string db_schema_;
  std::string table_name_;
  std::string prefix_;
  bool initialized_;
  static const char* protocol_;
};


/* Implementation of VariableStorage class */
VariableStorage::VariableStorage(const ros::NodeHandle& nh):
  nh_(nh)
{
  std::string key, storage_url;
  if (!nh_.searchParam("variable_storage_url", key))
  {
    return;
  }
  nh_.getParam(key, storage_url);

  std::string node_name = ros::this_node::getName();

  if (FileStorageBackend::matchUrl(storage_url))
  {
    backend_ = boost::make_shared<FileStorageBackend>(node_name, storage_url);
  }
  else if (MysqlStorageBackend::matchUrl(storage_url))
  {
    backend_ = boost::make_shared<MysqlStorageBackend>(node_name, storage_url);
  }
}

bool VariableStorage::hasVariable(const std::string& name)
{
  if (!backend_)
  {
    return false;
  }
  return backend_->hasVariable(name);
}

bool VariableStorage::getVariable(const std::string& name, std::string& value)
{
  if (!backend_)
  {
    return false;
  }
  return backend_->getVariable(name, value);
}

void VariableStorage::setVariable(const std::string& name, const std::string& value)
{
  if (!backend_)
  {
    return;
  }
  backend_->setVariable(name, value);
}

void VariableStorage::deleteVariable(const std::string& name)
{
  if (!backend_)
  {
    return;
  }
  backend_->deleteVariable(name);
}


/* Implementation of FileStorageBackend class */
namespace fs = boost::filesystem;

const char* FileStorageBackend::protocol_ = "file:///";

FileStorageBackend::FileStorageBackend(const std::string& node_name, const std::string& storage_url)
{
  if (!matchUrl(storage_url))
  {
    ROS_ERROR("[VariableStorage] File storage URL must start with \"file:///\". Storage backend is not used.");
    initialized_ = false;
  }
  else
  {
    size_t start = strlen(protocol_) - 1;  // copy the slash too
    folder_path_ = storage_url.substr(start) + node_name + "/";
    initialized_ = true;
  }
}

bool FileStorageBackend::hasVariable(const std::string& name)
{
  if (!initialized_) return false;

  try
  {
    fs::path path(folder_path_ + name);
    return fs::is_regular_file(path);
  }
  catch (fs::filesystem_error& ex)
  {
    ROS_ERROR("[VariableStorage] Filesystem error: %s", ex.what());
  }
  return false;
}

bool FileStorageBackend::getVariable(const std::string& name, std::string& value)
{
  if (!initialized_) return false;

  try
  {
    fs::path path(folder_path_ + name);
    if (fs::is_regular_file(path))
    {
      fs::ifstream ifs(path);

      std::ostringstream oss;
      oss << ifs.rdbuf();
      value = oss.str();
      return true;
    }
  }
  catch (fs::filesystem_error& ex)
  {
    ROS_ERROR("[VariableStorage] Filesystem error: %s", ex.what());
  }
  return false;
}

void FileStorageBackend::setVariable(const std::string& name, const std::string& value)
{
  if (!initialized_) return;

  try
  {
    fs::path path(folder_path_ + name);
    fs::create_directories(path.parent_path());
    fs::ofstream ofs(path);
    ofs << value;
  }
  catch (fs::filesystem_error& ex)
  {
    ROS_ERROR("[VariableStorage] Filesystem error: %s", ex.what());
  }
}

void FileStorageBackend::deleteVariable(const std::string& name)
{
  if (!initialized_) return;

  try
  {
    fs::path path(folder_path_ + name);
    fs::remove(path);
  }
  catch (fs::filesystem_error& ex)
  {
    ROS_ERROR("[VariableStorage] Filesystem error: %s", ex.what());
  }
}

bool FileStorageBackend::matchUrl(const std::string& storage_url)
{
  return storage_url.substr(0, strlen(protocol_)) == protocol_;
}


/* Implementation of MysqlStorageBackend class */
const char* MysqlStorageBackend::protocol_ = "mysql://";

MysqlStorageBackend::MysqlStorageBackend(const std::string& node_name, const std::string& url)
{
  initialized_ = false;
  if (!matchUrl(url))
  {
    ROS_ERROR("[VariableStorage] MySQL storage URL must start with \"mysql://\". Storage backend is not used.");
    return;
  }

  std::string storage_url = url.substr(strlen(protocol_));
  size_t db_schema_start = storage_url.find("/");
  if (db_schema_start == std::string::npos)
  {
    ROS_ERROR("[VariableStorage] Database schema name must be specified in the storage url after the host name, separated by a slash.");
    return;
  }

  db_schema_ = storage_url.substr(db_schema_start + 1);
  storage_url = storage_url.substr(0, db_schema_start);

  size_t table_name_start = db_schema_.find('#');
  if (table_name_start != std::string::npos)
  {
    table_name_ = db_schema_.substr(table_name_start + 1);
    db_schema_ = db_schema_.substr(0, table_name_start);
  }
  else
  {
    table_name_ = "variable_storage";
  }

  size_t db_host_start = storage_url.find('@');
  if (db_host_start != std::string::npos)
  {
    db_host_ = storage_url.substr(db_host_start + 1);
    storage_url = storage_url.substr(0, db_host_start);

    size_t db_pw_start = storage_url.find(':');
    if (db_pw_start != std::string::npos)
    {
      db_user_ = storage_url.substr(0, db_pw_start);
      db_password_ = storage_url.substr(db_pw_start + 1);
    }
    else
    {
      db_user_ = storage_url;
      db_password_ = "";
    }
  }
  else
  {
    db_host_ = storage_url;
    db_user_ = "";
    db_password_ = "";
  }

  prefix_ = node_name + "/";
  initialized_ = true;
}

bool MysqlStorageBackend::hasVariable(const std::string& name)
{
  if (!initialized_) return false;

  sql::Driver* driver = NULL;
  sql::Connection* conn = NULL;
  sql::PreparedStatement* pstmt = NULL;
  sql::ResultSet *res = NULL;

  int count = 0;
  try
  {
    driver = get_driver_instance();
    conn = driver->connect(db_host_, db_user_, db_password_);
    conn->setSchema(db_schema_);

    std::string query = "SELECT COUNT(*) FROM `" + table_name_ + "` WHERE `name` = (?)";
    pstmt = conn->prepareStatement(query.c_str());
    pstmt->setString(1, prefix_ + name);
    res = pstmt->executeQuery();

    while (res->next())
    {
      count = res->getInt(1);
    }
  }
  catch (sql::SQLException& ex)
  {
    ROS_ERROR("[VariableStorage] Database error: %s", ex.what());
  }

  if (res) delete res;
  if (pstmt) delete pstmt;
  if (conn) delete conn;
  if (driver) driver->threadEnd();
  return count;
}

bool MysqlStorageBackend::getVariable(const std::string& name, std::string& value)
{
  if (!initialized_) return false;

  sql::Driver* driver = NULL;
  sql::Connection* conn = NULL;
  sql::PreparedStatement* pstmt = NULL;
  sql::ResultSet *res = NULL;

  bool success = false;
  try
  {
    driver = get_driver_instance();
    conn = driver->connect(db_host_, db_user_, db_password_);
    conn->setSchema(db_schema_);

    std::string query = "SELECT `value` FROM `" + table_name_ + "` WHERE `name` = (?)";
    pstmt = conn->prepareStatement(query.c_str());
    pstmt->setString(1, prefix_ + name);
    res = pstmt->executeQuery();

    while (res->next())
    {
      value = res->getString(1);
      success = true;
    }
  }
  catch (sql::SQLException& ex)
  {
    ROS_ERROR("[VariableStorage] Database error: %s", ex.what());
  }

  if (res) delete res;
  if (pstmt) delete pstmt;
  if (conn) delete conn;
  if (driver) driver->threadEnd();
  return success;
}

void MysqlStorageBackend::setVariable(const std::string& name, const std::string& value)
{
  if (!initialized_) return;

  sql::Driver* driver = NULL;
  sql::Connection* conn = NULL;
  sql::Statement* stmt = NULL;
  sql::PreparedStatement* pstmt = NULL;

  try
  {
    driver = get_driver_instance();
    conn = driver->connect(db_host_, db_user_, db_password_);
    conn->setSchema(db_schema_);

    std::string create_table = "CREATE TABLE IF NOT EXISTS `" + table_name_ + "` (" +
                               "`name` VARCHAR(127) PRIMARY KEY NOT NULL," +
                               "`value` LONGTEXT NOT NULL," +
                               "`created` DATETIME NOT NULL," +
                               "`modified` DATETIME NOT NULL)";
    stmt = conn->createStatement();
    stmt->execute(create_table.c_str());

    std::string query = "INSERT INTO `" + table_name_ + "` VALUES (?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)" +
                        "ON DUPLICATE KEY UPDATE `value`=(?), `modified`=(CURRENT_TIMESTAMP)";
    pstmt = conn->prepareStatement(query.c_str());
    pstmt->setString(1, prefix_ + name);
    pstmt->setString(2, value);
    pstmt->setString(3, value);
    pstmt->executeUpdate();
  }
  catch (sql::SQLException& ex)
  {
    ROS_ERROR("[VariableStorage] Database error: %s", ex.what());
  }

  if (pstmt) delete pstmt;
  if (stmt) delete stmt;
  if (conn) delete conn;
  if (driver) driver->threadEnd();
}

void MysqlStorageBackend::deleteVariable(const std::string& name)
{
  if (!initialized_) return;

  sql::Driver* driver = NULL;
  sql::Connection* conn = NULL;
  sql::PreparedStatement* pstmt = NULL;

  try
  {
    driver = get_driver_instance();
    conn = driver->connect(db_host_, db_user_, db_password_);
    conn->setSchema(db_schema_);

    std::string query = "DELETE IGNORE FROM `" + table_name_ + "` WHERE `name` = (?)";
    pstmt = conn->prepareStatement(query.c_str());
    pstmt->setString(1, prefix_ + name);
    pstmt->executeUpdate();
  }
  catch (sql::SQLException& ex)
  {
    ROS_ERROR("[VariableStorage] Database error: %s", ex.what());
  }

  if (pstmt) delete pstmt;
  if (conn) delete conn;
  if (driver) driver->threadEnd();
}

bool MysqlStorageBackend::matchUrl(const std::string& storage_url)
{
  return storage_url.substr(0, strlen(protocol_)) == protocol_;
}

}  // namespace agv05
