#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

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

constexpr uint8_t REG_PSR_B2 = 0x00;
constexpr uint8_t REG_TMP_B2 = 0x03;
constexpr uint8_t REG_PRS_CFG = 0x06;
constexpr uint8_t REG_TMP_CFG = 0x07;
constexpr uint8_t REG_MEAS_CFG = 0x08;
constexpr uint8_t REG_CFG_REG = 0x09;
constexpr uint8_t REG_RESET = 0x0c;
constexpr uint8_t REG_PRODUCT_ID = 0x0d;
constexpr uint8_t REG_COEF = 0x10;

constexpr uint8_t SPL06_PRODUCT_ID = 0x10;
constexpr uint8_t MODE_BACKGROUND_PRESSURE_AND_TEMP = 0x07;
constexpr uint8_t SOFT_RESET = 0x09;
constexpr uint8_t SPI_READ = 0x80;

struct Options {
    std::string device = "/dev/spidev0.0";
    uint32_t speed_hz = 1000000;
    uint8_t mode = SPI_MODE_0;
    int samples = 0;
    int interval_ms = 500;
    double sea_level_hpa = 1013.25;
    int baseline_samples = 10;
};

struct Coefficients {
    int16_t c0 = 0;
    int16_t c1 = 0;
    int32_t c00 = 0;
    int32_t c10 = 0;
    int16_t c01 = 0;
    int16_t c11 = 0;
    int16_t c20 = 0;
    int16_t c21 = 0;
    int16_t c30 = 0;
};

struct Sample {
    int32_t raw_pressure = 0;
    int32_t raw_temperature = 0;
    double scaled_pressure = 0.0;
    double scaled_temperature = 0.0;
    double temperature_c = 0.0;
    double temperature_f = 0.0;
    double pressure_pa = 0.0;
    double pressure_hpa = 0.0;
    double pressure_kpa = 0.0;
    double pressure_atm = 0.0;
    double pressure_psi = 0.0;
    double pressure_mmhg = 0.0;
    double pressure_inhg = 0.0;
    double altitude_m = 0.0;
    double altitude_ft = 0.0;
    std::optional<double> relative_altitude_m;
};

[[noreturn]] void usage(const std::string &program, int exit_code) {
    std::ostream &out = exit_code == 0 ? std::cout : std::cerr;
    out
        << "Uso: " << program << " [opciones]\n\n"
        << "Lee un barómetro SPL06-001 por SPI y muestra valores de chip y altímetro.\n\n"
        << "Opciones:\n"
        << "  --device PATH          Dispositivo SPI (por defecto /dev/spidev0.0)\n"
        << "  --speed HZ            Velocidad SPI (por defecto 1000000)\n"
        << "  --samples N           Número de muestras; 0 = continuo (por defecto)\n"
        << "  --interval-ms N       Intervalo entre muestras (por defecto 500)\n"
        << "  --sea-level-hpa HPA   Presión de referencia a nivel del mar (por defecto 1013.25)\n"
        << "  --baseline-samples N  Muestras iniciales para altura relativa; 0 desactiva (por defecto 10)\n"
        << "  --help                Muestra esta ayuda\n";
    std::exit(exit_code);
}

int parse_int(const std::string &value, const std::string &name) {
    try {
        size_t pos = 0;
        int parsed = std::stoi(value, &pos, 10);
        if (pos != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return parsed;
    } catch (const std::exception &) {
        throw std::runtime_error("Valor inválido para " + name + ": " + value);
    }
}

double parse_double(const std::string &value, const std::string &name) {
    try {
        size_t pos = 0;
        double parsed = std::stod(value, &pos);
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
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Falta valor para " + name);
            }
            return argv[++i];
        };

        if (arg == "--device") {
            options.device = require_value(arg);
        } else if (arg == "--speed") {
            int speed = parse_int(require_value(arg), arg);
            if (speed <= 0) {
                throw std::runtime_error("--speed debe ser positivo");
            }
            options.speed_hz = static_cast<uint32_t>(speed);
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
        } else if (arg == "--sea-level-hpa") {
            options.sea_level_hpa = parse_double(require_value(arg), arg);
            if (options.sea_level_hpa <= 0.0) {
                throw std::runtime_error("--sea-level-hpa debe ser positivo");
            }
        } else if (arg == "--baseline-samples") {
            options.baseline_samples = parse_int(require_value(arg), arg);
            if (options.baseline_samples < 0) {
                throw std::runtime_error("--baseline-samples debe ser >= 0");
            }
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0], 0);
        } else {
            throw std::runtime_error("Opción desconocida: " + arg);
        }
    }
    return options;
}

int32_t sign_extend(uint32_t value, int bits) {
    const uint32_t sign_bit = 1u << (bits - 1);
    if ((value & sign_bit) == 0) {
        return static_cast<int32_t>(value);
    }
    const uint32_t mask = ~((1u << bits) - 1u);
    return static_cast<int32_t>(value | mask);
}

double scale_factor(uint8_t oversampling_code) {
    switch (oversampling_code & 0x07) {
    case 0: return 524288.0;
    case 1: return 1572864.0;
    case 2: return 3670016.0;
    case 3: return 7864320.0;
    case 4: return 253952.0;
    case 5: return 516096.0;
    case 6: return 1040384.0;
    case 7: return 2088960.0;
    default: throw std::logic_error("oversampling imposible");
    }
}

double altitude_meters(double pressure_hpa, double sea_level_hpa) {
    return 44330.0 * (1.0 - std::pow(pressure_hpa / sea_level_hpa, 0.19029495718));
}

class SpiDevice {
public:
    explicit SpiDevice(const Options &options) : fd_(open(options.device.c_str(), O_RDWR)) {
        if (fd_ < 0) {
            throw_errno("No se puede abrir " + options.device);
        }

        uint8_t mode = options.mode;
        uint8_t bits = 8;
        uint32_t speed = options.speed_hz;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd_, SPI_IOC_RD_MODE, &mode) < 0) {
            throw_errno("No se puede configurar modo SPI");
        }
        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
            throw_errno("No se puede configurar bits por palabra SPI");
        }
        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
            throw_errno("No se puede configurar velocidad SPI");
        }
        speed_hz_ = speed;
    }

    SpiDevice(const SpiDevice &) = delete;
    SpiDevice &operator=(const SpiDevice &) = delete;

    ~SpiDevice() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    uint8_t read8(uint8_t reg) {
        return read(reg, 1).at(0);
    }

    std::vector<uint8_t> read(uint8_t reg, size_t count) {
        std::vector<uint8_t> tx(count + 1, 0);
        std::vector<uint8_t> rx(count + 1, 0);
        tx[0] = reg | SPI_READ;
        transfer(tx, rx);
        return std::vector<uint8_t>(rx.begin() + 1, rx.end());
    }

    void write8(uint8_t reg, uint8_t value) {
        std::vector<uint8_t> tx{static_cast<uint8_t>(reg & ~SPI_READ), value};
        std::vector<uint8_t> rx(tx.size(), 0);
        transfer(tx, rx);
    }

    uint32_t speed_hz() const { return speed_hz_; }

private:
    int fd_ = -1;
    uint32_t speed_hz_ = 0;

    void transfer(const std::vector<uint8_t> &tx, std::vector<uint8_t> &rx) {
        spi_ioc_transfer transfer{};
        transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
        transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
        transfer.len = static_cast<uint32_t>(tx.size());
        transfer.speed_hz = speed_hz_;
        transfer.bits_per_word = 8;

        if (ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
            throw_errno("Fallo en transferencia SPI");
        }
    }

    [[noreturn]] void throw_errno(const std::string &message) const {
        throw std::runtime_error(message + ": " + std::strerror(errno));
    }
};

class Spl06 {
public:
    explicit Spl06(SpiDevice &spi) : spi_(spi) {}

    void begin() {
        spi_.write8(REG_CFG_REG, 0x00);  // Fuerza SPI de 4 hilos por si quedó activado SPI 3-wire.
        uint8_t id = product_id();
        if ((id & 0xf0) != SPL06_PRODUCT_ID) {
            throw std::runtime_error("SPL06-001 no detectado: product_id=0x" + hex_byte(id));
        }

        spi_.write8(REG_RESET, SOFT_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        spi_.write8(REG_CFG_REG, 0x00);
        wait_ready();
        coefficients_ = read_coefficients();

        // 4 Hz y sobremuestreo x8 para presión y temperatura. TMP_EXT=1 según hoja de datos.
        pressure_oversampling_code_ = 0x03;
        temperature_oversampling_code_ = 0x03;
        spi_.write8(REG_PRS_CFG, static_cast<uint8_t>((0x02 << 4) | pressure_oversampling_code_));
        spi_.write8(REG_TMP_CFG, static_cast<uint8_t>(0x80 | (0x02 << 4) | temperature_oversampling_code_));
        spi_.write8(REG_CFG_REG, 0x00);
        spi_.write8(REG_MEAS_CFG, MODE_BACKGROUND_PRESSURE_AND_TEMP);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    uint8_t product_id() { return spi_.read8(REG_PRODUCT_ID); }
    uint8_t status() { return spi_.read8(REG_MEAS_CFG); }
    uint8_t pressure_config() { return spi_.read8(REG_PRS_CFG); }
    uint8_t temperature_config() { return spi_.read8(REG_TMP_CFG); }
    uint8_t config() { return spi_.read8(REG_CFG_REG); }
    const Coefficients &coefficients() const { return coefficients_; }

    Sample read_sample(double sea_level_hpa, std::optional<double> baseline_pressure_hpa) {
        auto pressure_bytes = spi_.read(REG_PSR_B2, 3);
        auto temperature_bytes = spi_.read(REG_TMP_B2, 3);

        const int32_t raw_pressure = raw24(pressure_bytes);
        const int32_t raw_temperature = raw24(temperature_bytes);
        const double scaled_pressure = raw_pressure / scale_factor(pressure_oversampling_code_);
        const double scaled_temperature = raw_temperature / scale_factor(temperature_oversampling_code_);

        const double temperature_c = coefficients_.c0 * 0.5 + coefficients_.c1 * scaled_temperature;
        const double pressure_pa = coefficients_.c00
            + scaled_pressure * (coefficients_.c10 + scaled_pressure * (coefficients_.c20 + scaled_pressure * coefficients_.c30))
            + scaled_temperature * coefficients_.c01
            + scaled_temperature * scaled_pressure * (coefficients_.c11 + scaled_pressure * coefficients_.c21);
        const double pressure_hpa = pressure_pa / 100.0;
        const double absolute_altitude_m = altitude_meters(pressure_hpa, sea_level_hpa);

        Sample sample;
        sample.raw_pressure = raw_pressure;
        sample.raw_temperature = raw_temperature;
        sample.scaled_pressure = scaled_pressure;
        sample.scaled_temperature = scaled_temperature;
        sample.temperature_c = temperature_c;
        sample.temperature_f = temperature_c * 9.0 / 5.0 + 32.0;
        sample.pressure_pa = pressure_pa;
        sample.pressure_hpa = pressure_hpa;
        sample.pressure_kpa = pressure_pa / 1000.0;
        sample.pressure_atm = pressure_pa / 101325.0;
        sample.pressure_psi = pressure_pa / 6894.757293168;
        sample.pressure_mmhg = pressure_pa / 133.322387415;
        sample.pressure_inhg = pressure_pa / 3386.38815789;
        sample.altitude_m = absolute_altitude_m;
        sample.altitude_ft = absolute_altitude_m * 3.280839895;
        if (baseline_pressure_hpa.has_value()) {
            sample.relative_altitude_m = altitude_meters(pressure_hpa, *baseline_pressure_hpa);
        }
        return sample;
    }

private:
    SpiDevice &spi_;
    Coefficients coefficients_;
    uint8_t pressure_oversampling_code_ = 0;
    uint8_t temperature_oversampling_code_ = 0;

    void wait_ready() {
        for (int attempt = 0; attempt < 100; ++attempt) {
            const uint8_t ready = spi_.read8(REG_MEAS_CFG) & 0xc0;
            if (ready == 0xc0) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        throw std::runtime_error("El SPL06-001 no informó sensor/coefs listos");
    }

    Coefficients read_coefficients() {
        auto b = spi_.read(REG_COEF, 18);
        Coefficients c;
        c.c0 = static_cast<int16_t>(sign_extend((static_cast<uint32_t>(b[0]) << 4) | (b[1] >> 4), 12));
        c.c1 = static_cast<int16_t>(sign_extend(((static_cast<uint32_t>(b[1]) & 0x0f) << 8) | b[2], 12));
        c.c00 = sign_extend((static_cast<uint32_t>(b[3]) << 12) | (static_cast<uint32_t>(b[4]) << 4) | (b[5] >> 4), 20);
        c.c10 = sign_extend(((static_cast<uint32_t>(b[5]) & 0x0f) << 16) | (static_cast<uint32_t>(b[6]) << 8) | b[7], 20);
        c.c01 = static_cast<int16_t>((static_cast<uint16_t>(b[8]) << 8) | b[9]);
        c.c11 = static_cast<int16_t>((static_cast<uint16_t>(b[10]) << 8) | b[11]);
        c.c20 = static_cast<int16_t>((static_cast<uint16_t>(b[12]) << 8) | b[13]);
        c.c21 = static_cast<int16_t>((static_cast<uint16_t>(b[14]) << 8) | b[15]);
        c.c30 = static_cast<int16_t>((static_cast<uint16_t>(b[16]) << 8) | b[17]);
        return c;
    }

    static int32_t raw24(const std::vector<uint8_t> &bytes) {
        return sign_extend((static_cast<uint32_t>(bytes[0]) << 16) | (static_cast<uint32_t>(bytes[1]) << 8) | bytes[2], 24);
    }

    static std::string hex_byte(uint8_t value) {
        std::ostringstream out;
        out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(value);
        return out.str();
    }
};

void print_header(const Options &options, const SpiDevice &spi, Spl06 &sensor) {
    const auto &c = sensor.coefficients();
    std::cout << "SPL06-001 altímetro por SPI\n"
              << "  device=" << options.device << " speed_hz=" << spi.speed_hz() << " mode=0\n"
              << "  product_id=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(sensor.product_id())
              << std::dec << " status=0x" << std::hex << std::setw(2) << static_cast<int>(sensor.status())
              << " prs_cfg=0x" << std::setw(2) << static_cast<int>(sensor.pressure_config())
              << " tmp_cfg=0x" << std::setw(2) << static_cast<int>(sensor.temperature_config())
              << " cfg=0x" << std::setw(2) << static_cast<int>(sensor.config()) << std::dec << "\n"
              << "  coef: c0=" << c.c0 << " c1=" << c.c1 << " c00=" << c.c00 << " c10=" << c.c10
              << " c01=" << c.c01 << " c11=" << c.c11 << " c20=" << c.c20
              << " c21=" << c.c21 << " c30=" << c.c30 << "\n"
              << "  referencia_nivel_mar=" << options.sea_level_hpa << " hPa\n";
}

void print_sample(int index, const Sample &sample) {
    std::cout << std::fixed << std::setprecision(3)
              << "muestra=" << index
              << " raw_p=" << sample.raw_pressure
              << " raw_t=" << sample.raw_temperature
              << " p_sc=" << sample.scaled_pressure
              << " t_sc=" << sample.scaled_temperature
              << " temp=" << sample.temperature_c << " C/" << sample.temperature_f << " F"
              << " presion=" << sample.pressure_pa << " Pa"
              << " (" << sample.pressure_hpa << " hPa, " << sample.pressure_kpa << " kPa, "
              << sample.pressure_atm << " atm, " << sample.pressure_psi << " psi, "
              << sample.pressure_mmhg << " mmHg, " << sample.pressure_inhg << " inHg)"
              << " altura=" << sample.altitude_m << " m/" << sample.altitude_ft << " ft";
    if (sample.relative_altitude_m.has_value()) {
        std::cout << " altura_relativa=" << *sample.relative_altitude_m << " m";
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const Options options = parse_args(argc, argv);
        SpiDevice spi(options);
        Spl06 sensor(spi);
        sensor.begin();
        print_header(options, spi, sensor);

        std::optional<double> baseline_pressure_hpa;
        if (options.baseline_samples > 0) {
            double sum = 0.0;
            for (int i = 0; i < options.baseline_samples; ++i) {
                sum += sensor.read_sample(options.sea_level_hpa, std::nullopt).pressure_hpa;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            baseline_pressure_hpa = sum / options.baseline_samples;
            std::cout << std::fixed << std::setprecision(3)
                      << "  baseline_relativo=" << *baseline_pressure_hpa << " hPa ("
                      << options.baseline_samples << " muestras)\n";
        }

        const bool continuous = options.samples == 0;
        for (int i = 1; continuous || i <= options.samples; ++i) {
            print_sample(i, sensor.read_sample(options.sea_level_hpa, baseline_pressure_hpa));
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
