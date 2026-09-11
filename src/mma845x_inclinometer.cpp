#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint8_t REG_STATUS = 0x00;
constexpr uint8_t REG_OUT_X_MSB = 0x01;
constexpr uint8_t REG_WHO_AM_I = 0x0d;
constexpr uint8_t REG_XYZ_DATA_CFG = 0x0e;
constexpr uint8_t REG_CTRL_REG1 = 0x2a;
constexpr uint8_t REG_CTRL_REG2 = 0x2b;

struct Options {
    std::vector<std::string> buses = {"/dev/i2c-1", "/dev/i2c-2"};
    std::optional<uint8_t> address;
    int samples = 0;
    int interval_ms = 150;
    bool clear_screen = true;
};

struct DeviceInfo {
    std::string model;
    uint8_t who_am_i = 0;
    int bits = 0;
    double counts_per_g = 1.0;
};

struct AccelSample {
    uint8_t status = 0;
    int16_t raw_x = 0;
    int16_t raw_y = 0;
    int16_t raw_z = 0;
    double x_g = 0.0;
    double y_g = 0.0;
    double z_g = 0.0;
    double magnitude_g = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
};

[[noreturn]] void usage(const std::string &program, int exit_code) {
    std::ostream &out = exit_code == 0 ? std::cout : std::cerr;
    out << "Uso: " << program << " [opciones]\n\n"
        << "Lee un acelerómetro MMA845x por I2C y muestra inclinación en tiempo real.\n"
        << "Nota: la familia MMA845x usa bus I2C, no SPI.\n\n"
        << "Opciones:\n"
        << "  --bus PATH             Bus I2C; se puede repetir (por defecto /dev/i2c-1 y /dev/i2c-2)\n"
        << "  --address HEX          Dirección 7-bit I2C, normalmente 0x1c o 0x1d\n"
        << "  --samples N           Número de muestras; 0 = continuo (por defecto)\n"
        << "  --interval-ms N       Intervalo entre muestras (por defecto 150)\n"
        << "  --no-clear            No limpia pantalla entre muestras\n"
        << "  --help                Muestra esta ayuda\n";
    std::exit(exit_code);
}

int parse_int(const std::string &value, const std::string &name) {
    try {
        size_t pos = 0;
        int parsed = std::stoi(value, &pos, 0);
        if (pos != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return parsed;
    } catch (const std::exception &) {
        throw std::runtime_error("Valor inválido para " + name + ": " + value);
    }
}

Options parse_args(int argc, char **argv) {
    Options options;
    bool custom_bus = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Falta valor para " + name);
            }
            return argv[++i];
        };

        if (arg == "--bus") {
            if (!custom_bus) {
                options.buses.clear();
                custom_bus = true;
            }
            options.buses.push_back(require_value(arg));
        } else if (arg == "--address") {
            int value = parse_int(require_value(arg), arg);
            if (value < 0x03 || value > 0x77) {
                throw std::runtime_error("--address debe ser una dirección I2C 7-bit entre 0x03 y 0x77");
            }
            options.address = static_cast<uint8_t>(value);
        } else if (arg == "--samples") {
            options.samples = parse_int(require_value(arg), arg);
            if (options.samples < 0) {
                throw std::runtime_error("--samples debe ser >= 0");
            }
        } else if (arg == "--interval-ms") {
            options.interval_ms = parse_int(require_value(arg), arg);
            if (options.interval_ms < 1) {
                throw std::runtime_error("--interval-ms debe ser >= 1");
            }
        } else if (arg == "--no-clear") {
            options.clear_screen = false;
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0], 0);
        } else {
            throw std::runtime_error("Opción desconocida: " + arg);
        }
    }
    if (options.buses.empty()) {
        throw std::runtime_error("Debe indicarse al menos un --bus");
    }
    return options;
}

std::optional<DeviceInfo> device_info_for_id(uint8_t who_am_i) {
    switch (who_am_i) {
    case 0x1a: return DeviceInfo{"MMA8451Q", who_am_i, 14, 4096.0};
    case 0x2a: return DeviceInfo{"MMA8452Q", who_am_i, 12, 1024.0};
    case 0x3a: return DeviceInfo{"MMA8453Q", who_am_i, 10, 256.0};
    default: return std::nullopt;
    }
}

class I2cDevice {
public:
    I2cDevice(std::string bus_path, uint8_t address)
        : bus_path_(std::move(bus_path)), address_(address), fd_(open(bus_path_.c_str(), O_RDWR)) {
        if (fd_ < 0) {
            throw_errno("No se puede abrir " + bus_path_);
        }
        if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
            throw_errno("No se puede seleccionar dirección I2C");
        }
    }

    I2cDevice(const I2cDevice &) = delete;
    I2cDevice &operator=(const I2cDevice &) = delete;

    I2cDevice(I2cDevice &&other) noexcept
        : bus_path_(std::move(other.bus_path_)), address_(other.address_), fd_(other.fd_) {
        other.fd_ = -1;
    }

    ~I2cDevice() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    uint8_t read8(uint8_t reg) const {
        return smbus_read_byte_data(reg);
    }

    std::vector<uint8_t> read(uint8_t reg, size_t count) const {
        std::vector<uint8_t> values;
        values.reserve(count);
        for (size_t offset = 0; offset < count; ++offset) {
            values.push_back(read8(static_cast<uint8_t>(reg + offset)));
        }
        return values;
    }

    void write8(uint8_t reg, uint8_t value) const {
        smbus_write_byte_data(reg, value);
    }

    const std::string &bus_path() const { return bus_path_; }
    uint8_t address() const { return address_; }

private:
    std::string bus_path_;
    uint8_t address_ = 0;
    int fd_ = -1;

    [[noreturn]] void throw_errno(const std::string &message) const {
        std::ostringstream out;
        out << message << " (" << bus_path_ << " addr=0x" << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<int>(address_) << "): " << std::strerror(errno);
        throw std::runtime_error(out.str());
    }

    void smbus_access(char read_write, uint8_t command, int size, i2c_smbus_data *data) const {
        i2c_smbus_ioctl_data args{};
        args.read_write = read_write;
        args.command = command;
        args.size = size;
        args.data = data;
        if (ioctl(fd_, I2C_SMBUS, &args) < 0) {
            throw_errno("Fallo SMBus/I2C");
        }
    }

    uint8_t smbus_read_byte_data(uint8_t command) const {
        i2c_smbus_data data{};
        smbus_access(I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
        return data.byte;
    }

    void smbus_write_byte_data(uint8_t command, uint8_t value) const {
        i2c_smbus_data data{};
        data.byte = value;
        smbus_access(I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
    }
};

class Mma845x {
public:
    Mma845x(I2cDevice device, DeviceInfo info) : device_(std::move(device)), info_(std::move(info)) {}

    void begin() const {
        uint8_t ctrl1 = device_.read8(REG_CTRL_REG1);
        device_.write8(REG_CTRL_REG1, ctrl1 & ~0x01);  // standby para configurar.
        device_.write8(REG_XYZ_DATA_CFG, 0x00);        // ±2 g.
        device_.write8(REG_CTRL_REG2, 0x00);           // auto-sleep y reset desactivados.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        device_.write8(REG_CTRL_REG1, 0x19);           // activo, ODR 100 Hz, low-noise.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    AccelSample read_sample() const {
        auto bytes = device_.read(REG_STATUS, 7);
        const int shift = 16 - info_.bits;
        AccelSample sample;
        sample.status = bytes[0];
        sample.raw_x = static_cast<int16_t>((static_cast<uint16_t>(bytes[1]) << 8) | bytes[2]) >> shift;
        sample.raw_y = static_cast<int16_t>((static_cast<uint16_t>(bytes[3]) << 8) | bytes[4]) >> shift;
        sample.raw_z = static_cast<int16_t>((static_cast<uint16_t>(bytes[5]) << 8) | bytes[6]) >> shift;
        sample.x_g = sample.raw_x / info_.counts_per_g;
        sample.y_g = sample.raw_y / info_.counts_per_g;
        sample.z_g = sample.raw_z / info_.counts_per_g;
        sample.magnitude_g = std::sqrt(sample.x_g * sample.x_g + sample.y_g * sample.y_g + sample.z_g * sample.z_g);
        sample.roll_deg = std::atan2(sample.y_g, sample.z_g) * 180.0 / M_PI;
        sample.pitch_deg = std::atan2(-sample.x_g, std::sqrt(sample.y_g * sample.y_g + sample.z_g * sample.z_g)) * 180.0 / M_PI;
        return sample;
    }

    const DeviceInfo &info() const { return info_; }
    const I2cDevice &device() const { return device_; }
    uint8_t ctrl1() const { return device_.read8(REG_CTRL_REG1); }
    uint8_t data_cfg() const { return device_.read8(REG_XYZ_DATA_CFG); }

private:
    I2cDevice device_;
    DeviceInfo info_;
};

std::optional<Mma845x> detect(const Options &options) {
    std::vector<uint8_t> addresses;
    if (options.address.has_value()) {
        addresses.push_back(*options.address);
    } else {
        addresses = {0x1c, 0x1d};
    }

    for (const auto &bus : options.buses) {
        for (uint8_t address : addresses) {
            try {
                I2cDevice device(bus, address);
                uint8_t who = device.read8(REG_WHO_AM_I);
                auto info = device_info_for_id(who);
                if (info.has_value()) {
                    return Mma845x(std::move(device), *info);
                }
            } catch (const std::exception &) {
                // Probar siguiente bus/dirección.
            }
        }
    }
    return std::nullopt;
}

std::string bar(double value, double min_value, double max_value, int width = 31) {
    value = std::max(min_value, std::min(max_value, value));
    double normalized = (value - min_value) / (max_value - min_value);
    int marker = static_cast<int>(std::lround(normalized * (width - 1)));
    int center = width / 2;
    std::string out;
    out.reserve(width);
    for (int i = 0; i < width; ++i) {
        if (i == marker) {
            out.push_back('O');
        } else if (i == center) {
            out.push_back('|');
        } else {
            out.push_back('-');
        }
    }
    return out;
}

std::string attitude_grid(double roll_deg, double pitch_deg) {
    constexpr int width = 33;
    constexpr int height = 13;
    constexpr int center_x = width / 2;
    constexpr int center_y = height / 2;
    std::array<std::array<char, width>, height> grid{};
    for (auto &line : grid) {
        line.fill(' ');
    }

    for (int x = 0; x < width; ++x) {
        grid[center_y][x] = '-';
    }
    for (int y = 0; y < height; ++y) {
        grid[y][center_x] = '|';
    }
    grid[center_y][center_x] = '+';

    double slope = std::tan(roll_deg * M_PI / 180.0);
    int pitch_offset = static_cast<int>(std::lround(std::max(-45.0, std::min(45.0, pitch_deg)) / 45.0 * (height / 2 - 1)));
    for (int x = 0; x < width; ++x) {
        double relative_x = (x - center_x) / static_cast<double>(center_x);
        int y = center_y - pitch_offset + static_cast<int>(std::lround(relative_x * slope * 5.0));
        if (0 <= y && y < height) {
            grid[y][x] = '=';
        }
    }

    std::ostringstream out;
    out << '+' << std::string(width, '-') << "+\n";
    for (const auto &line : grid) {
        out << '|';
        for (char ch : line) {
            out << ch;
        }
        out << "|\n";
    }
    out << '+' << std::string(width, '-') << '+';
    return out.str();
}

void print_header(const Mma845x &sensor) {
    std::cout << "MMA845x inclinómetro I2C\n"
              << "  bus=" << sensor.device().bus_path()
              << " addr=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(sensor.device().address())
              << " who_am_i=0x" << std::setw(2) << static_cast<int>(sensor.info().who_am_i)
              << " ctrl1=0x" << std::setw(2) << static_cast<int>(sensor.ctrl1())
              << " xyz_data_cfg=0x" << std::setw(2) << static_cast<int>(sensor.data_cfg())
              << std::dec << std::setfill(' ') << "\n"
              << "  modelo=" << sensor.info().model << " resolucion=" << sensor.info().bits
              << " bits rango=+-2g\n";
}

void print_sample(int index, const AccelSample &sample) {
    std::cout << std::fixed << std::setprecision(3)
              << "muestra=" << index
              << " status=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(sample.status)
              << std::dec << std::setfill(' ')
              << " raw_x=" << sample.raw_x << " raw_y=" << sample.raw_y << " raw_z=" << sample.raw_z << "\n"
              << "  g_x=" << sample.x_g << " g_y=" << sample.y_g << " g_z=" << sample.z_g
              << " |g|=" << sample.magnitude_g << "\n"
              << "  roll=" << sample.roll_deg << " deg  pitch=" << sample.pitch_deg << " deg\n"
              << "  roll  [-90..+90] " << bar(sample.roll_deg, -90.0, 90.0) << "\n"
              << "  pitch [-90..+90] " << bar(sample.pitch_deg, -90.0, 90.0) << "\n"
              << attitude_grid(sample.roll_deg, sample.pitch_deg) << "\n";
}

}  // namespace

int main(int argc, char **argv) {
    try {
        Options options = parse_args(argc, argv);
        auto sensor = detect(options);
        if (!sensor.has_value()) {
            throw std::runtime_error(
                "No se encontró MMA8451/8452/8453 en las direcciones 0x1c/0x1d. "
                "La familia MMA845x es I2C; revisa SDA/SCL, VCC/GND y prueba --bus/--address.");
        }
        sensor->begin();

        const bool continuous = options.samples == 0;
        for (int i = 1; continuous || i <= options.samples; ++i) {
            if (options.clear_screen) {
                std::cout << "\033[2J\033[H";
            }
            print_header(*sensor);
            print_sample(i, sensor->read_sample());
            std::cout.flush();
            if (continuous || i < options.samples) {
                std::this_thread::sleep_for(std::chrono::milliseconds(options.interval_ms));
            }
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
