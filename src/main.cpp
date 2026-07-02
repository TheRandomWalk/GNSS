#include "gnss/acquisition.hpp"
#include "gnss/pvt.hpp"
#include "gnss/tracking.hpp"

#if defined(GNSS_HAS_UHD)
#include <uhd/stream.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string args;
    std::string antenna = "RX2";
    std::string csv;
    std::string iq_file;
    std::string iq_format = "fc32";
    std::string pvt_file;
    std::string pvt_out_file;
    std::string track_csv;
    double rate = 2.046e6;
    double frequency = 1575.42e6;
    double gain = 45.0;
    double bandwidth = 2.4e6;
    double seconds = 0.08;
    double threshold = 2.5;
    double doppler_min = -5000.0;
    double doppler_max = 5000.0;
    double doppler_step = 500.0;
    int first_prn = 1;
    int last_prn = 32;
    int track_ms = 0;
    std::optional<int> track_prn;
    bool help = false;
};

void print_usage() {
    std::cout
        << "gnss - receptor GPS L1 C/A por software para Ettus B200 y archivos IQ\n\n"
        << "Uso:\n"
        << "  gnss [opciones]\n\n"
        << "Modos:\n"
        << "  Sin --iq ni --pvt      captura desde UHD/B200 y ejecuta adquisicion\n"
        << "  --iq FILE             lee IQ desde archivo y ejecuta adquisicion/tracking\n"
        << "  --pvt FILE            resuelve posicion desde CSV de observaciones\n\n"
        << "Opciones RF/adquisicion:\n"
        << "  --args STR             argumentos UHD, ej. \"type=b200\"\n"
        << "  --antenna RX2|TX/RX    entrada RF del B200 (default RX2)\n"
        << "  --gain DB              ganancia RX (default 45)\n"
        << "  --frequency HZ         frecuencia RX (default 1575420000)\n"
        << "  --rate SAMPLES         tasa de muestreo (default 2046000)\n"
        << "  --bandwidth HZ         ancho de banda RX (default 2400000)\n"
        << "  --seconds N            segundos a capturar (default 0.08)\n"
        << "  --prn A-B              rango PRN GPS C/A (default 1-32)\n"
        << "  --doppler A:B:C        min:max:paso en Hz (default -5000:5000:500)\n"
        << "  --threshold N          umbral peak/promedio (default 2.5)\n"
        << "  --csv FILE             escribe resultados de adquisicion CSV\n\n"
        << "Opciones IQ/tracking:\n"
        << "  --iq FILE              archivo IQ offline\n"
        << "  --iq-format fc32|sc16|cs8 formato IQ (default fc32)\n"
        << "  --track-ms N           epochs de tracking de 1 ms a exportar\n"
        << "  --track-prn N          PRN a seguir; si se omite usa el mejor lock\n"
        << "  --track-csv FILE       escribe tracking CSV\n\n"
        << "Opciones PVT:\n"
        << "  --pvt FILE             CSV: prn,x_m,y_m,z_m,pseudorange_m[,sigma_m]\n"
        << "  --pvt-out FILE         escribe solucion y residuos CSV\n"
        << "  --help                 muestra esta ayuda\n";
}

double parse_double_arg(const std::string& name, const char* value) {
    if (value == nullptr) {
        throw std::invalid_argument("Falta valor para " + name);
    }
    return std::stod(value);
}

int parse_int_arg(const std::string& name, const char* value) {
    if (value == nullptr) {
        throw std::invalid_argument("Falta valor para " + name);
    }
    return std::stoi(value);
}

std::pair<int, int> parse_prn_range(const std::string& value) {
    const auto dash = value.find('-');
    if (dash == std::string::npos) {
        const int prn = std::stoi(value);
        return {prn, prn};
    }
    return {std::stoi(value.substr(0, dash)), std::stoi(value.substr(dash + 1))};
}

void parse_doppler(const std::string& value, Options& options) {
    const auto first = value.find(':');
    const auto second = value.find(':', first == std::string::npos ? first : first + 1);
    if (first == std::string::npos || second == std::string::npos) {
        throw std::invalid_argument("--doppler debe tener formato min:max:paso");
    }
    options.doppler_min = std::stod(value.substr(0, first));
    options.doppler_max = std::stod(value.substr(first + 1, second - first - 1));
    options.doppler_step = std::stod(value.substr(second + 1));
}

Options parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                throw std::invalid_argument("Falta valor para " + arg);
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--args") {
            options.args = next();
        } else if (arg == "--antenna") {
            options.antenna = next();
        } else if (arg == "--csv") {
            options.csv = next();
        } else if (arg == "--iq") {
            options.iq_file = next();
        } else if (arg == "--iq-format") {
            options.iq_format = next();
        } else if (arg == "--pvt") {
            options.pvt_file = next();
        } else if (arg == "--pvt-out") {
            options.pvt_out_file = next();
        } else if (arg == "--track-csv") {
            options.track_csv = next();
        } else if (arg == "--track-ms") {
            options.track_ms = parse_int_arg(arg, next());
        } else if (arg == "--track-prn") {
            options.track_prn = parse_int_arg(arg, next());
        } else if (arg == "--gain") {
            options.gain = parse_double_arg(arg, next());
        } else if (arg == "--frequency") {
            options.frequency = parse_double_arg(arg, next());
        } else if (arg == "--rate") {
            options.rate = parse_double_arg(arg, next());
        } else if (arg == "--bandwidth") {
            options.bandwidth = parse_double_arg(arg, next());
        } else if (arg == "--seconds") {
            options.seconds = parse_double_arg(arg, next());
        } else if (arg == "--threshold") {
            options.threshold = parse_double_arg(arg, next());
        } else if (arg == "--prn") {
            const auto [first, last] = parse_prn_range(next());
            options.first_prn = first;
            options.last_prn = last;
        } else if (arg == "--doppler") {
            parse_doppler(next(), options);
        } else {
            throw std::invalid_argument("Opcion desconocida: " + arg);
        }
    }
    return options;
}

void configure_uhd_images(char* executable) {
    std::error_code path_error;
    const auto images = std::filesystem::absolute(executable, path_error).parent_path() / "uhd_images";
    if (!path_error && std::filesystem::exists(images)) {
#ifdef _WIN32
        _putenv_s("UHD_IMAGES_DIR", images.string().c_str());
#else
        setenv("UHD_IMAGES_DIR", images.string().c_str(), 1);
#endif
    }
}

std::string lowercase(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

std::vector<std::complex<float>> read_iq_file(const Options& options) {
    std::ifstream input(options.iq_file, std::ios::binary);
    if (!input) {
        throw std::runtime_error("No se pudo abrir IQ: " + options.iq_file);
    }

    const std::string format = lowercase(options.iq_format);
    std::vector<std::complex<float>> samples;

    if (format == "fc32") {
        while (true) {
            float i = 0.0F;
            float q = 0.0F;
            input.read(reinterpret_cast<char*>(&i), sizeof(float));
            input.read(reinterpret_cast<char*>(&q), sizeof(float));
            if (!input) {
                break;
            }
            samples.emplace_back(i, q);
        }
    } else if (format == "sc16") {
        while (true) {
            std::int16_t i = 0;
            std::int16_t q = 0;
            input.read(reinterpret_cast<char*>(&i), sizeof(std::int16_t));
            input.read(reinterpret_cast<char*>(&q), sizeof(std::int16_t));
            if (!input) {
                break;
            }
            samples.emplace_back(static_cast<float>(i) / 32768.0F, static_cast<float>(q) / 32768.0F);
        }
    } else if (format == "cs8") {
        while (true) {
            std::int8_t i = 0;
            std::int8_t q = 0;
            input.read(reinterpret_cast<char*>(&i), sizeof(std::int8_t));
            input.read(reinterpret_cast<char*>(&q), sizeof(std::int8_t));
            if (!input) {
                break;
            }
            samples.emplace_back(static_cast<float>(i) / 128.0F, static_cast<float>(q) / 128.0F);
        }
    } else {
        throw std::invalid_argument("--iq-format debe ser fc32, sc16 o cs8");
    }

    std::cout << "IQ: " << samples.size() << " muestras desde " << options.iq_file << " (" << format << ")\n";
    return samples;
}

std::vector<std::complex<float>> capture_samples(const Options& options) {
#if defined(GNSS_HAS_UHD)
    auto usrp = uhd::usrp::multi_usrp::make(options.args);
    usrp->set_rx_rate(options.rate);
    usrp->set_rx_gain(options.gain);
    usrp->set_rx_bandwidth(options.bandwidth);
    usrp->set_rx_antenna(options.antenna);
    usrp->set_rx_freq(uhd::tune_request_t(options.frequency));

    std::cout << "B200: " << usrp->get_rx_freq() / 1e6 << " MHz, "
              << usrp->get_rx_rate() / 1e6 << " MS/s, "
              << usrp->get_rx_gain() << " dB, antena " << usrp->get_rx_antenna() << "\n";

    uhd::stream_args_t stream_args("fc32", "sc16");
    auto stream = usrp->get_rx_stream(stream_args);
    const std::size_t target = static_cast<std::size_t>(std::max(1.0, options.seconds * options.rate));
    std::vector<std::complex<float>> samples(target);
    std::vector<std::complex<float>> buffer(stream->get_max_num_samps());
    uhd::rx_metadata_t metadata;

    uhd::stream_cmd_t command(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    command.stream_now = true;
    stream->issue_stream_cmd(command);

    std::size_t written = 0;
    std::uint64_t overflows = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (written < target && std::chrono::steady_clock::now() < deadline) {
        const auto received = stream->recv(buffer.data(), buffer.size(), metadata, 0.5);
        if (metadata.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            ++overflows;
            continue;
        }
        if (metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            continue;
        }
        const auto copy_count = std::min<std::size_t>(received, target - written);
        std::copy_n(buffer.begin(), copy_count, samples.begin() + static_cast<std::ptrdiff_t>(written));
        written += copy_count;
    }

    command.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    stream->issue_stream_cmd(command);
    samples.resize(written);
    std::cout << "Muestras: " << samples.size() << ", overflows: " << overflows << "\n";
    return samples;
#else
    (void)options;
    throw std::runtime_error("Este binario fue compilado sin UHD. Usa --iq FILE o recompila con UHD disponible.");
#endif
}

void write_csv(const std::string& path, const std::vector<gnss::AcquisitionResult>& results) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("No se pudo escribir CSV: " + path);
    }
    output << "prn,detected,doppler_hz,code_phase_samples,metric,cn0_dbhz\n";
    for (const auto& result : results) {
        output << result.prn << ','
               << (result.detected ? "true" : "false") << ','
               << result.doppler_hz << ','
               << result.code_phase_samples << ','
               << result.metric << ','
               << result.cn0_estimate_dbhz << '\n';
    }
}

std::vector<std::string> split_csv_line(std::string line) {
    std::replace(line.begin(), line.end(), ';', ',');
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        const auto first = field.find_first_not_of(" \t\r\n");
        const auto last = field.find_last_not_of(" \t\r\n");
        fields.push_back(first == std::string::npos ? std::string{} : field.substr(first, last - first + 1));
    }
    return fields;
}

bool starts_with_number(const std::string& text) {
    if (text.empty()) {
        return false;
    }
    const char c = text.front();
    return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.';
}

std::vector<gnss::SatelliteObservation> read_pvt_observations(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("No se pudo abrir PVT CSV: " + path);
    }

    std::vector<gnss::SatelliteObservation> observations;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const auto fields = split_csv_line(line);
        if (fields.size() < 5) {
            continue;
        }
        if (!starts_with_number(fields[0])) {
            continue;
        }
        gnss::SatelliteObservation obs;
        obs.prn = std::stoi(fields[0]);
        obs.satellite_ecef_m.x = std::stod(fields[1]);
        obs.satellite_ecef_m.y = std::stod(fields[2]);
        obs.satellite_ecef_m.z = std::stod(fields[3]);
        obs.pseudorange_m = std::stod(fields[4]);
        obs.sigma_m = fields.size() >= 6 && !fields[5].empty() ? std::stod(fields[5]) : 1.0;
        observations.push_back(obs);
    }
    return observations;
}

void print_pvt_solution(const gnss::PVTSolution& solution) {
    std::cout << "\nPVT: " << (solution.valid ? "VALIDA" : "INVALIDA") << " - " << solution.message << "\n";
    if (!solution.valid) {
        return;
    }
    std::cout << std::fixed << std::setprecision(6)
              << "Lat/Lon/Alt: " << solution.receiver_lla.latitude_deg << ", "
              << solution.receiver_lla.longitude_deg << ", "
              << std::setprecision(3) << solution.receiver_lla.height_m << " m\n"
              << "ECEF: " << solution.receiver_ecef_m.x << ", "
              << solution.receiver_ecef_m.y << ", "
              << solution.receiver_ecef_m.z << " m\n"
              << "Clock bias: " << solution.clock_bias_m << " m / "
              << std::setprecision(9) << solution.clock_bias_s << " s\n"
              << std::setprecision(3)
              << "RMS residual: " << solution.rms_residual_m << " m\n"
              << "DOP GDOP/PDOP/HDOP/VDOP/TDOP: "
              << solution.gdop << " / " << solution.pdop << " / "
              << solution.hdop << " / " << solution.vdop << " / " << solution.tdop << "\n";
}

void write_pvt_solution(const std::string& path, const gnss::PVTSolution& solution,
                        const std::vector<gnss::SatelliteObservation>& observations) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("No se pudo escribir PVT CSV: " + path);
    }
    output << "valid,lat_deg,lon_deg,height_m,ecef_x_m,ecef_y_m,ecef_z_m,clock_bias_m,clock_bias_s,gdop,pdop,hdop,vdop,tdop,rms_residual_m\n";
    output << (solution.valid ? "true" : "false") << ','
           << solution.receiver_lla.latitude_deg << ','
           << solution.receiver_lla.longitude_deg << ','
           << solution.receiver_lla.height_m << ','
           << solution.receiver_ecef_m.x << ','
           << solution.receiver_ecef_m.y << ','
           << solution.receiver_ecef_m.z << ','
           << solution.clock_bias_m << ','
           << solution.clock_bias_s << ','
           << solution.gdop << ','
           << solution.pdop << ','
           << solution.hdop << ','
           << solution.vdop << ','
           << solution.tdop << ','
           << solution.rms_residual_m << "\n\n";
    output << "prn,residual_m,pseudorange_m,sigma_m\n";
    for (std::size_t i = 0; i < observations.size(); ++i) {
        output << observations[i].prn << ','
               << (i < solution.residuals_m.size() ? solution.residuals_m[i] : 0.0) << ','
               << observations[i].pseudorange_m << ','
               << observations[i].sigma_m << '\n';
    }
}

void write_tracking_csv(const std::string& path, const std::vector<gnss::TrackingPoint>& points) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("No se pudo escribir tracking CSV: " + path);
    }
    output << "epoch,code_phase_samples,doppler_hz,prompt_i,prompt_q,early_mag,prompt_mag,late_mag,dll_discriminator,cn0_dbhz\n";
    for (const auto& point : points) {
        output << point.epoch << ','
               << point.code_phase_samples << ','
               << point.doppler_hz << ','
               << point.prompt_i << ','
               << point.prompt_q << ','
               << point.early_magnitude << ','
               << point.prompt_magnitude << ','
               << point.late_magnitude << ','
               << point.dll_discriminator << ','
               << point.cn0_estimate_dbhz << '\n';
    }
}

int solve_pvt_mode(const Options& options) {
    const auto observations = read_pvt_observations(options.pvt_file);
    const auto solution = gnss::solve_pvt(observations);
    print_pvt_solution(solution);
    if (!options.pvt_out_file.empty()) {
        write_pvt_solution(options.pvt_out_file, solution, observations);
        std::cout << "PVT CSV: " << options.pvt_out_file << "\n";
    }
    return solution.valid ? 0 : 2;
}

} // namespace

int main(int argc, char** argv) {
    try {
        configure_uhd_images(argv[0]);
        const auto options = parse_args(argc, argv);
        if (options.help) {
            print_usage();
            return 0;
        }

        if (!options.pvt_file.empty()) {
            return solve_pvt_mode(options);
        }

        const auto samples = options.iq_file.empty() ? capture_samples(options) : read_iq_file(options);
        gnss::AcquisitionConfig config;
        config.sample_rate_hz = options.rate;
        config.doppler_min_hz = options.doppler_min;
        config.doppler_max_hz = options.doppler_max;
        config.doppler_step_hz = options.doppler_step;
        config.threshold = options.threshold;

        const auto results = gnss::acquire(samples, config, options.first_prn, options.last_prn);
        std::cout << "\nPRN  Estado  Doppler(Hz)  FaseCodigo  Metrica  C/N0 est\n";
        for (const auto& result : results) {
            std::cout << std::setw(3) << result.prn << "  "
                      << std::setw(6) << (result.detected ? "LOCK" : "-") << "  "
                      << std::setw(11) << std::fixed << std::setprecision(0) << result.doppler_hz << "  "
                      << std::setw(10) << result.code_phase_samples << "  "
                      << std::setw(7) << std::setprecision(2) << result.metric << "  "
                      << std::setw(8) << std::setprecision(1) << result.cn0_estimate_dbhz << "\n";
        }

        if (!options.csv.empty()) {
            write_csv(options.csv, results);
            std::cout << "\nCSV: " << options.csv << "\n";
        }

        const auto locks = std::count_if(results.begin(), results.end(), [](const auto& r) { return r.detected; });
        if (options.track_ms > 0 || !options.track_csv.empty()) {
            auto selected = results.end();
            if (options.track_prn.has_value()) {
                selected = std::find_if(results.begin(), results.end(), [&](const auto& r) { return r.prn == *options.track_prn; });
            } else {
                selected = std::find_if(results.begin(), results.end(), [](const auto& r) { return r.detected; });
            }
            if (selected == results.end()) {
                throw std::runtime_error("No hay PRN valido para tracking; usa --track-prn o baja --threshold");
            }

            gnss::TrackingConfig tracking_config;
            tracking_config.sample_rate_hz = options.rate;
            tracking_config.max_epochs = options.track_ms > 0 ? options.track_ms : 100;
            gnss::TrackingState state;
            state.prn = selected->prn;
            state.code_phase_samples = static_cast<double>(selected->code_phase_samples);
            state.doppler_hz = selected->doppler_hz;
            const auto points = gnss::track(samples, tracking_config, state);
            std::cout << "\nTracking PRN " << state.prn << ": " << points.size() << " epochs de 1 ms\n";
            if (!options.track_csv.empty()) {
                write_tracking_csv(options.track_csv, points);
                std::cout << "Tracking CSV: " << options.track_csv << "\n";
            }
        }

        if (locks < 4) {
            std::cout << "\nAviso: para posicionamiento 3D hacen falta al menos 4 satelites, tracking estable y datos NAV/efemerides.\n";
        }
        return locks == 0 ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
