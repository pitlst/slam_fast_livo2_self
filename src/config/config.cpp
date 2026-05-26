#include "general.hpp"
#include "config.hpp"

using namespace hsm;

config::config(): _path(std::filesystem::path(PROJECT_PATH) / "config" / "config.toml")
{
    this->parser();
}

config::config(const std::filesystem::path& input_path): _path(input_path)
{
    this->parser();
}

void config::parser()
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
