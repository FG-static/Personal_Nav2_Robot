#include "my_tunnel_guidance/tunnel_guidance_search.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace my_tunnel_guidance {

namespace {

constexpr int kImageScale = 4;
constexpr double kEpsilon = 1e-9;

using CellOffset = std::pair<int, int>;

constexpr CellOffset kNeighborOffsets[] = {
    CellOffset{1, 0},
    CellOffset{1, 1},
    CellOffset{0, 1},
    CellOffset{-1, 1},
    CellOffset{-1, 0},
    CellOffset{-1, -1},
    CellOffset{0, -1},
    CellOffset{1, -1}};

struct DemoOptions {
    std::filesystem::path output_dir = "/tmp/tunnel_guidance_demo";
};

std::size_t cellIndex(const TunnelGrid & grid, int mx, int my)
{
    return static_cast<std::size_t>(my) *
        static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(mx);
}

bool isInside(const TunnelGrid & grid, int mx, int my)
{
    return mx >= 0 && my >= 0 && mx < grid.width && my < grid.height;
}

Eigen::Vector2d cellCenter(const TunnelGrid & grid, int mx, int my)
{
    return grid.origin + Eigen::Vector2d(
        (static_cast<double>(mx) + 0.5) * grid.resolution,
        (static_cast<double>(my) + 0.5) * grid.resolution);
}

bool worldToCell(
    const TunnelGrid & grid,
    const Eigen::Vector2d & point,
    int & mx,
    int & my)
{
    const double cell_x = (point.x() - grid.origin.x()) / grid.resolution;
    const double cell_y = (point.y() - grid.origin.y()) / grid.resolution;
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
        cell_x < 0.0 || cell_y < 0.0 ||
        cell_x >= static_cast<double>(grid.width) ||
        cell_y >= static_cast<double>(grid.height)) {
        return false;
    }

    mx = static_cast<int>(std::floor(cell_x));
    my = static_cast<int>(std::floor(cell_y));
    return isInside(grid, mx, my);
}

cv::Rect cellRect(const TunnelGrid & grid, int mx, int my)
{
    const int image_y = grid.height - 1 - my;
    return cv::Rect(
        mx * kImageScale,
        image_y * kImageScale,
        kImageScale,
        kImageScale);
}

bool worldToPixel(
    const TunnelGrid & grid,
    const Eigen::Vector2d & point,
    cv::Point & pixel)
{
    int mx = 0;
    int my = 0;
    if (!worldToCell(grid, point, mx, my)) {
        return false;
    }

    const cv::Rect rectangle = cellRect(grid, mx, my);
    pixel = cv::Point(
        rectangle.x + rectangle.width / 2,
        rectangle.y + rectangle.height / 2);
    return true;
}

double corridorCenterY(double x)
{
    if (x <= 2.0) {
        return 0.0;
    }
    return 0.72 * std::sin(0.55 * (x - 2.0));
}

TunnelGrid makeDemoGrid()
{
    TunnelGrid grid;
    grid.width = 160;
    grid.height = 80;
    grid.resolution = 0.10;
    grid.origin = Eigen::Vector2d(-1.0, -4.0);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height),
        GridState::Unknown);

    // The corridor is known only locally. Everything outside it stays Unknown,
    // so the far end and the two sides form a visible frontier.
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const Eigen::Vector2d point = cellCenter(grid, mx, my);
            const double x = point.x();
            if (x < -0.1 || x > 13.8) {
                continue;
            }

            const double lateral_distance =
                std::abs(point.y() - corridorCenterY(x));
            const std::size_t index = cellIndex(grid, mx, my);
            if (lateral_distance <= 1.20) {
                grid.states[index] = GridState::Free;
            } else if (lateral_distance <= 1.35) {
                grid.states[index] = GridState::Occupied;
            }
        }
    }

    // A box slightly offset from the local centerline leaves a visible
    // detour while preserving a traversable passage on both sides.
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const Eigen::Vector2d point = cellCenter(grid, mx, my);
            if (point.x() < 5.2 || point.x() > 6.5) {
                continue;
            }

            const double box_center_y = corridorCenterY(point.x()) - 0.20;
            if (std::abs(point.y() - box_center_y) <= 0.35) {
                grid.states[cellIndex(grid, mx, my)] = GridState::Occupied;
            }
        }
    }

    // The robot starts at (0, 0), matching TunnelGuidanceSearch::searchGrid().
    int start_x = 0;
    int start_y = 0;
    if (worldToCell(grid, Eigen::Vector2d::Zero(), start_x, start_y)) {
        grid.states[cellIndex(grid, start_x, start_y)] = GridState::Free;
    }
    return grid;
}

cv::Scalar stateColor(GridState state)
{
    switch (state) {
        case GridState::Free:
            return cv::Scalar(245, 245, 245);
        case GridState::Occupied:
            return cv::Scalar(25, 25, 25);
        case GridState::Unknown:
        default:
            return cv::Scalar(128, 128, 128);
    }
}

cv::Mat renderBaseGrid(const TunnelGrid & grid)
{
    cv::Mat image(
        grid.height * kImageScale,
        grid.width * kImageScale,
        CV_8UC3,
        cv::Scalar(128, 128, 128));
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            cv::rectangle(
                image,
                cellRect(grid, mx, my),
                stateColor(grid.states[cellIndex(grid, mx, my)]),
                cv::FILLED);
        }
    }
    return image;
}

bool isFrontierCell(const TunnelGrid & grid, int mx, int my)
{
    if (!isInside(grid, mx, my) ||
        grid.states[cellIndex(grid, mx, my)] != GridState::Free) {
        return false;
    }

    for (const CellOffset & offset : kNeighborOffsets) {
        const int neighbor_x = mx + offset.first;
        const int neighbor_y = my + offset.second;
        if (isInside(grid, neighbor_x, neighbor_y) &&
            grid.states[cellIndex(grid, neighbor_x, neighbor_y)] ==
            GridState::Unknown) {
            return true;
        }
    }
    return false;
}

std::size_t countState(const TunnelGrid & grid, GridState state)
{
    return static_cast<std::size_t>(std::count(
        grid.states.begin(), grid.states.end(), state));
}

std::size_t countFrontierCells(const TunnelGrid & grid)
{
    std::size_t count = 0U;
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            if (isFrontierCell(grid, mx, my)) {
                ++count;
            }
        }
    }
    return count;
}

cv::Mat renderSearchFrame(
    const TunnelGrid & grid,
    const SearchDebugFrame & frame)
{
    cv::Mat image = renderBaseGrid(grid);
    if (frame.width != grid.width || frame.height != grid.height ||
        frame.costs.size() != grid.states.size() ||
        frame.closed.size() != grid.states.size()) {
        return image;
    }

    double min_cost = std::numeric_limits<double>::infinity();
    double max_cost = 0.0;
    for (std::size_t index = 0U; index < frame.costs.size(); ++index) {
        if (frame.closed[index] == 0U || !std::isfinite(frame.costs[index])) {
            continue;
        }
        min_cost = std::min(min_cost, frame.costs[index]);
        max_cost = std::max(max_cost, frame.costs[index]);
    }

    if (!std::isfinite(min_cost)) {
        return image;
    }

    const double cost_range = std::max(max_cost - min_cost, kEpsilon);
    cv::Mat normalized(
        grid.height,
        grid.width,
        CV_8UC1,
        cv::Scalar(0));
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const std::size_t index = cellIndex(grid, mx, my);
            if (frame.closed[index] == 0U ||
                !std::isfinite(frame.costs[index]) ||
                grid.states[index] == GridState::Occupied ||
                grid.states[index] == GridState::Unknown) {
                continue;
            }

            const double value =
                (frame.costs[index] - min_cost) / cost_range;
            const int image_y = grid.height - 1 - my;
            normalized.at<unsigned char>(image_y, mx) = static_cast<unsigned char>(
                std::clamp(value, 0.0, 1.0) * 255.0);
        }
    }

    cv::Mat colors;
    cv::applyColorMap(normalized, colors, cv::COLORMAP_JET);
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const std::size_t index = cellIndex(grid, mx, my);
            if (frame.closed[index] == 0U ||
                !std::isfinite(frame.costs[index]) ||
                grid.states[index] == GridState::Occupied ||
                grid.states[index] == GridState::Unknown) {
                continue;
            }

            const int image_y = grid.height - 1 - my;
            const cv::Vec3b color = colors.at<cv::Vec3b>(image_y, mx);
            cv::rectangle(image, cellRect(grid, mx, my), cv::Scalar(
                    color[0], color[1], color[2]), cv::FILLED);
        }
    }

    std::ostringstream label;
    label << "Dijkstra expanded: " << frame.expanded_count;
    cv::putText(
        image,
        label.str(),
        cv::Point(8, 18),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);
    return image;
}

cv::Mat renderEsdf(
    const TunnelGrid & grid,
    const SearchDebugFrame & frame)
{
    cv::Mat image = renderBaseGrid(grid);
    if (frame.width != grid.width || frame.height != grid.height ||
        frame.esdf_distances.size() != grid.states.size()) {
        return image;
    }

    double max_distance = 0.0;
    for (std::size_t index = 0U; index < frame.esdf_distances.size(); ++index) {
        if (grid.states[index] == GridState::Free &&
            std::isfinite(frame.esdf_distances[index])) {
            max_distance = std::max(max_distance, frame.esdf_distances[index]);
        }
    }
    max_distance = std::max(max_distance, kEpsilon);

    cv::Mat normalized(
        grid.height,
        grid.width,
        CV_8UC1,
        cv::Scalar(0));
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const std::size_t index = cellIndex(grid, mx, my);
            if (grid.states[index] != GridState::Free ||
                !std::isfinite(frame.esdf_distances[index])) {
                continue;
            }

            const double value = frame.esdf_distances[index] / max_distance;
            const int image_y = grid.height - 1 - my;
            normalized.at<unsigned char>(image_y, mx) = static_cast<unsigned char>(
                std::clamp(value, 0.0, 1.0) * 255.0);
        }
    }

    cv::Mat colors;
    cv::applyColorMap(normalized, colors, cv::COLORMAP_JET);
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const std::size_t index = cellIndex(grid, mx, my);
            if (grid.states[index] != GridState::Free) {
                continue;
            }

            const int image_y = grid.height - 1 - my;
            const cv::Vec3b color = colors.at<cv::Vec3b>(image_y, mx);
            cv::rectangle(image, cellRect(grid, mx, my), cv::Scalar(
                    color[0], color[1], color[2]), cv::FILLED);
        }
    }

    cv::putText(
        image,
        "ESDF clearance (blue=low, red=high)",
        cv::Point(8, 18),
        cv::FONT_HERSHEY_SIMPLEX,
        0.40,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);
    return image;
}

void drawFrontierCandidates(
    cv::Mat & image,
    const TunnelGrid & grid,
    const SearchDebugFrame & frame)
{
    const bool has_search_data =
        frame.closed.size() == grid.states.size() &&
        frame.costs.size() == grid.states.size();

    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            if (!isFrontierCell(grid, mx, my)) {
                continue;
            }

            const std::size_t index = cellIndex(grid, mx, my);
            const bool reachable =
                has_search_data && frame.closed[index] != 0U &&
                std::isfinite(frame.costs[index]);
            const cv::Scalar color = reachable ?
                cv::Scalar(0, 215, 255) : cv::Scalar(0, 140, 255);
            cv::rectangle(image, cellRect(grid, mx, my), color, cv::FILLED);
        }
    }
}

cv::Mat renderFrontierCandidates(
    const TunnelGrid & grid,
    const SearchDebugFrame & frame)
{
    cv::Mat image = renderBaseGrid(grid);
    drawFrontierCandidates(image, grid, frame);

    cv::putText(
        image,
        "frontier candidates (yellow=reachable)",
        cv::Point(8, 18),
        cv::FONT_HERSHEY_SIMPLEX,
        0.40,
        cv::Scalar(20, 20, 20),
        1,
        cv::LINE_AA);
    return image;
}

void drawWorldCircle(
    cv::Mat & image,
    const TunnelGrid & grid,
    const Eigen::Vector2d & point,
    const cv::Scalar & color,
    int radius,
    int thickness)
{
    cv::Point pixel;
    if (worldToPixel(grid, point, pixel)) {
        cv::circle(image, pixel, radius, color, thickness, cv::LINE_AA);
    }
}

void drawWorldPolyline(
    cv::Mat & image,
    const TunnelGrid & grid,
    const std::vector<Eigen::Vector3d> & points,
    const cv::Scalar & color,
    int thickness)
{
    std::vector<cv::Point> pixels;
    pixels.reserve(points.size());
    for (const Eigen::Vector3d & point : points) {
        cv::Point pixel;
        if (worldToPixel(grid, point.head<2>(), pixel)) {
            pixels.push_back(pixel);
        }
    }
    if (pixels.size() >= 2U) {
        cv::polylines(
            image,
            pixels,
            false,
            color,
            thickness,
            cv::LINE_AA);
    }
}

cv::Mat renderFinalGuidance(
    const TunnelGrid & grid,
    const SearchDebugFrame & frame,
    const TunnelGuidanceSearchResult & result)
{
    cv::Mat image = renderBaseGrid(grid);
    drawFrontierCandidates(image, grid, frame);
    drawWorldPolyline(
        image,
        grid,
        result.path,
        cv::Scalar(0, 180, 0),
        2);

    drawWorldCircle(
        image,
        grid,
        Eigen::Vector2d::Zero(),
        cv::Scalar(255, 0, 0),
        5,
        cv::FILLED);
    drawWorldCircle(
        image,
        grid,
        result.goal.head<2>(),
        cv::Scalar(0, 0, 255),
        5,
        cv::FILLED);

    cv::Point goal_pixel;
    if (worldToPixel(grid, result.goal.head<2>(), goal_pixel)) {
        const Eigen::Vector2d tangent = result.goal_tangent.head<2>();
        const Eigen::Vector2d arrow_end = result.goal.head<2>() +
            0.70 * tangent;
        cv::Point arrow_pixel;
        if (worldToPixel(grid, arrow_end, arrow_pixel)) {
            cv::arrowedLine(
                image,
                goal_pixel,
                arrow_pixel,
                cv::Scalar(0, 0, 255),
                2,
                cv::LINE_AA,
                0,
                0.25);
        }
    }

    cv::putText(
        image,
        "guidance path / start / inspection goal",
        cv::Point(8, 18),
        cv::FONT_HERSHEY_SIMPLEX,
        0.40,
        cv::Scalar(20, 20, 20),
        1,
        cv::LINE_AA);
    return image;
}

bool parseArguments(int argc, char ** argv, DemoOptions & options)
{
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: tunnel_guidance_demo [--output-dir DIR]\n";
            return false;
        }
        if (argument == "--output-dir") {
            if (index + 1 >= argc) {
                std::cerr << "--output-dir requires a directory\n";
                return false;
            }
            options.output_dir = argv[++index];
            continue;
        }

        std::cerr << "Unknown argument: " << argument << "\n";
        return false;
    }
    return true;
}

bool createOutputDirectories(const DemoOptions & options)
{
    std::error_code error;
    std::filesystem::create_directories(options.output_dir, error);
    if (error) {
        std::cerr << "Failed to create output directory " <<
            options.output_dir << ": " << error.message() << "\n";
        return false;
    }
    std::filesystem::create_directories(options.output_dir / "frames", error);
    if (error) {
        std::cerr << "Failed to create frame directory: " <<
            error.message() << "\n";
        return false;
    }
    return true;
}

std::filesystem::path framePath(
    const std::filesystem::path & output_dir,
    std::size_t frame_index)
{
    std::ostringstream filename;
    filename << "frame_" << std::setw(5) << std::setfill('0') << frame_index << ".png";
    return output_dir / "frames" / filename.str();
}

double computePathLength(const std::vector<Eigen::Vector3d> & path)
{
    double length = 0.0;
    for (std::size_t index = 1U; index < path.size(); ++index) {
        length += (path[index] - path[index - 1U]).head<2>().norm();
    }
    return length;
}

bool writeImage(
    const std::filesystem::path & path,
    const cv::Mat & image)
{
    if (image.empty() || !cv::imwrite(path.string(), image)) {
        std::cerr << "Failed to write image: " << path << "\n";
        return false;
    }
    return true;
}

}  // namespace

}  // namespace my_tunnel_guidance

int main(int argc, char ** argv)
{
    using namespace my_tunnel_guidance;

    DemoOptions options;
    if (!parseArguments(argc, argv, options)) {
        return 1;
    }
    if (!createOutputDirectories(options)) {
        return 1;
    }

    const TunnelGrid grid = makeDemoGrid();
    const std::size_t free_count = countState(grid, GridState::Free);
    const std::size_t occupied_count = countState(grid, GridState::Occupied);
    const std::size_t unknown_count = countState(grid, GridState::Unknown);
    const std::size_t frontier_count = countFrontierCells(grid);

    TunnelGuidanceSearchParams params;
    params.resolution = grid.resolution;
    params.min_x = grid.origin.x();
    params.max_x = grid.origin.x() +
        static_cast<double>(grid.width) * grid.resolution;
    params.half_width = 4.0;
    params.robot_clearance = 0.32;
    params.clearance_weight = 2.0;
    params.clearance_decay = 0.50;
    params.minimum_frontier_distance = 3.0;
    params.goal_distance = 5.0;
    params.debug_expansion_interval = 75U;

    const cv::Size image_size(
        grid.width * kImageScale,
        grid.height * kImageScale);
    const std::filesystem::path video_path =
        options.output_dir / "dijkstra_search.avi";
    cv::VideoWriter video_writer;
    video_writer.open(
        video_path.string(),
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        15.0,
        image_size,
        true);
    const bool video_enabled = video_writer.isOpened();
    if (!video_enabled) {
        std::cerr << "VideoWriter unavailable; writing search frames as PNG files\n";
    }

    SearchDebugFrame latest_frame;
    std::size_t frame_index = 0U;
    const SearchDebugCallback debug_callback =
        [&grid, &video_writer, video_enabled, &options, &latest_frame, &frame_index](
        const SearchDebugFrame & frame) {
            const cv::Mat image = renderSearchFrame(grid, frame);
            if (video_enabled) {
                video_writer.write(image);
            } else {
                writeImage(framePath(options.output_dir, frame_index), image);
            }
            ++frame_index;
            latest_frame = frame;
        };

    std::cout << "Tunnel guidance 2D demo\n";
    std::cout << "Grid: " << grid.width << " x " << grid.height <<
        ", resolution=" << grid.resolution << " m\n";
    std::cout << "Cells: free=" << free_count <<
        ", occupied=" << occupied_count <<
        ", unknown=" << unknown_count << "\n";
    std::cout << "Frontier candidates: " << frontier_count << "\n";

    const TunnelGuidanceSearch search(params);
    const auto search_start = std::chrono::steady_clock::now();
    const TunnelGuidanceSearchResult result =
        search.searchGrid(grid, debug_callback);
    const auto search_end = std::chrono::steady_clock::now();
    const double search_ms = std::chrono::duration<double, std::milli>(
        search_end - search_start).count();

    if (video_enabled) {
        video_writer.release();
    }

    if (latest_frame.width == 0 || latest_frame.height == 0) {
        std::cerr << "The search did not emit a debug frame\n";
        latest_frame.width = grid.width;
        latest_frame.height = grid.height;
        latest_frame.resolution = grid.resolution;
        latest_frame.origin = grid.origin;
    }

    const cv::Mat input_image = renderBaseGrid(grid);
    const cv::Mat esdf_image = renderEsdf(grid, latest_frame);
    const cv::Mat cost_image = renderSearchFrame(grid, latest_frame);
    const cv::Mat frontier_image = renderFrontierCandidates(grid, latest_frame);
    const cv::Mat final_image = renderFinalGuidance(grid, latest_frame, result);

    bool output_ok = true;
    output_ok = writeImage(options.output_dir / "input_costmap.png", input_image) && output_ok;
    output_ok = writeImage(options.output_dir / "esdf_heatmap.png", esdf_image) && output_ok;
    output_ok = writeImage(options.output_dir / "search_cost.png", cost_image) && output_ok;
    output_ok = writeImage(
        options.output_dir / "frontier_candidates.png",
        frontier_image) && output_ok;
    output_ok = writeImage(options.output_dir / "final_guidance.png", final_image) && output_ok;

    const double path_length = computePathLength(result.path);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Dijkstra expanded: " << latest_frame.expanded_count << " cells\n";
    std::cout << "Search time: " << search_ms << " ms\n";
    std::cout << "Guidance path: " << result.path.size() <<
        " points, length=" << path_length << " m\n";
    std::cout << "Inspection goal: (" << result.goal.x() << ", " <<
        result.goal.y() << "), clearance=" << result.goal_clearance << " m\n";
    std::cout << "Goal tangent: (" << result.goal_tangent.x() << ", " <<
        result.goal_tangent.y() << ")\n";
    std::cout << "Result valid: " << (result.valid ? "true" : "false") << "\n";
    std::cout << "Output directory: " << options.output_dir << "\n";
    if (video_enabled) {
        std::cout << "Search animation: " << video_path <<
            " (" << frame_index << " frames)\n";
    } else {
        std::cout << "Search animation frames: " <<
            options.output_dir / "frames" <<
            " (" << frame_index << " frames)\n";
    }

    return result.valid && output_ok ? 0 : 2;
}
