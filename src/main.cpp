#include "gnss/acquisition.hpp"

#include <uhd/stream.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string args;
    std::string antenna = "RX2";
    std::string csv;
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
    bool help = false;
};

void print_usage() {
    std::cout
        << "gnss - receptor GPS L1 C/A por software para Ettus B200\n\n"
        << "Uso:\n"
        << "  gnss [opciones]\n\n"
        << "Opciones:\n"
        << "  --args STR             argumentos UHD, ej. \"type=b200\"\n"
        << "  --antenna RX2|TX/RX    entrada RF del B200 (default RX2)\n"
        << "  --gain DB              ganancia RX (default 45)\n"
        << "  --rate SAMPLES         tasa de muestreo (default 2046000)\n"
        << "  --bandwidth HZ         ancho de banda RX (default 2400000)\n"
        << "  --seconds N            segundos a capturar (default 0.08)\n"
        << "  --prn A-B              rango PRN GPS C/A (default 1-32)\n"
        << "  --doppler A:B:C        min:max:paso en Hz (default -5000:5000:500)\n"
        << "  --threshold N          umbral peak/promedio (default 2.5)\n"
        << "  --csv FILE             escribe resultados CSV\n"
        << "  --help                 muestra esta ayuda\n";
}

double parse_double_arg(const std::string& name, const char* value) {
    if (value == nullptr) {
        throw std::invalid_argument("Falta valor para " + name);
    }
    return std::stod(value);
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
        } else if (arg == "--gain") {
            options.gain = parse_double_arg(arg, next());
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

std::vector<std::complex<float>> capture_samples(const Options& options) {
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
}

void write_csv(const std::string& path, const std::vector<gnss::AcquisitionResult>& results) {
    std::ofstream output(path);
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

} // namespace

int main(int argc, char** argv) {
    try {
        configure_uhd_images(argv[0]);
        const auto options = parse_args(argc, argv);
        if (options.help) {
            print_usage();
            return 0;
        }

        auto samples = capture_samples(options);
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
        if (locks < 4) {
            std::cout << "\nAviso: para posicionamiento 3D hacen falta al menos 4 satelites y efemerides validas.\n";
        }
        return locks == 0 ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
