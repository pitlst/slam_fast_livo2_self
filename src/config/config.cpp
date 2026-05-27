#include "general.hpp"
#include "config.hpp"

using namespace hsm;

camera_config::camera_config(): _path(std::filesystem::path(PROJECT_PATH) / "config" / "camera.toml")
{
    this->parser();
}

camera_config::camera_config(const std::filesystem::path& input_path): _path(input_path)
{
    this->parser();
}

void camera_config::parser()
{
    check_file_exist(this->_path);
    toml::table toml_data = toml::parse_file(this->_path.string());
    this->device_id       = parser_config_item<int>(this->_path, toml_data, "device_id");
    this->width           = parser_config_item<int>(this->_path, toml_data, "width");
    this->height          = parser_config_item<int>(this->_path, toml_data, "height");
    this->offset_x        = parser_config_item<int>(this->_path, toml_data, "offset_x");
    this->offset_y        = parser_config_item<int>(this->_path, toml_data, "offset_y");
    this->exposure        = parser_config_item<int>(this->_path, toml_data, "exposure");
    this->gain            = parser_config_item<int>(this->_path, toml_data, "gain");
}

mid360_config::mid360_config(): _path(std::filesystem::path(PROJECT_PATH) / "config" / "mid360.toml")
{
    this->parser();
}

mid360_config::mid360_config(const std::filesystem::path& input_path): _path(input_path)
{
    this->parser();
}

void mid360_config::parser()
{
    check_file_exist(this->_path);
    toml::table toml_data = toml::parse_file(this->_path.string());
    this->host_ip         = parser_config_item<std::string>(this->_path, toml_data, "host_ip");
    this->multicast_ip    = parser_config_item<std::string>(this->_path, toml_data, "multicast_ip");
    this->lidar_type      = parser_config_item<std::string>(this->_path, toml_data, "lidar_type");
    this->cmd_port        = parser_config_item<int>(this->_path, toml_data, "cmd_port");
    this->push_port       = parser_config_item<int>(this->_path, toml_data, "push_port");
    this->point_port      = parser_config_item<int>(this->_path, toml_data, "point_port");
    this->imu_port        = parser_config_item<int>(this->_path, toml_data, "imu_port");
    this->log_port        = parser_config_item<int>(this->_path, toml_data, "log_port");
}
