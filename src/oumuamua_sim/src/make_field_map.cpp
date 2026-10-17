/// sotoba_ros と同じフィールドのモデルから、走査面 (LiDAR の高さ) で見える壁の地図を作る。
///
/// - `<name>.pgm` / `<name>.yaml`: 占有格子地図 (nav2_map_server の形式)。AMCL や Nav2 のコストマップ用
/// - `<name>.pcd`: 壁の表面の点群 (z = 0、ASCII)。small_gicp などの点群の位置合わせ用
///
/// フィールドの中に格子状に置いた視点から走査面で光線を飛ばし、当たった点を集める。
/// 実際の LiDAR が見られる面だけが地図に入る (壁の裏や、厚みの中は入らない)。
///
/// sotoba_node と同じパラメータファイルを読ませること (wall_height、include_notes、include_shelf、lidar_height)。
/// このノード自身のパラメータは `map.*` (sotoba_node のパラメータと名前がぶつからないように)。
/// 書き終えたら終了する。

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sotoba/math/se3.hpp>
#include <sotoba/math/vec.hpp>

#include "sotoba_ros/objects.hpp"

namespace {
	namespace math = sotoba::math;
	using sotoba::math::UVec3;
	using sotoba::math::Vec3;
	using sotoba_ros::Surface;

	struct Point2 {
		double x;
		double y;
	};

	class MakeFieldMap final : public rclcpp::Node {
	public:
		MakeFieldMap() : rclcpp::Node("make_field_map") {
			const sotoba_ros::ObjectsConfig defaults{};
			sotoba_ros::ObjectsConfig field{};
			field.wall_height = static_cast<float>(
				this->declare_parameter<double>("wall_height", static_cast<double>(defaults.wall_height))
			);
			field.wall_thickness = static_cast<float>(this->declare_parameter<double>(
				"wall_thickness", static_cast<double>(defaults.wall_thickness)
			));
			field.include_notes = this->declare_parameter<bool>("include_notes", defaults.include_notes);
			field.include_shelf = this->declare_parameter<bool>("include_shelf", defaults.include_shelf);
			this->height_ =
				this->declare_parameter<double>("lidar_height", static_cast<double>(defaults.lidar_height));
			// robot_sim と同じく、フィールド座標系での姿勢を得るために原点・高さ 0・正立の LiDAR を指定する
			field.start_x = 0.f;
			field.start_y = 0.f;
			field.start_yaw = 0.f;
			field.lidar_height = 0.f;
			field.lidar_upside_down = false;
			for (const auto& object : sotoba_ros::make_objects(field)) {
				for (const auto& surface : object.surfaces) {
					Surface s = surface;
					std::visit([&](auto& v) { v.apply_se3(object.initial_pose); }, s);
					this->surfaces_.push_back(s);
				}
			}

			this->output_dir_ = this->declare_parameter<std::string>("map.output_dir", ".");
			this->name_ = this->declare_parameter<std::string>("map.name", "field");
			this->resolution_ = this->declare_parameter<double>("map.resolution", 0.01);
			this->point_spacing_ = this->declare_parameter<double>("map.point_spacing", 0.005);
			this->viewpoint_step_ = this->declare_parameter<double>("map.viewpoint_step", 0.2);
			this->ray_num_ = this->declare_parameter<std::int64_t>("map.ray_num", 2880);
			this->range_max_ = this->declare_parameter<double>("map.range_max", 10.0);
			// 視点を置く範囲 (フィールドの外周壁の内側)
			this->half_x_ = this->declare_parameter<double>("map.field_half_x", 2.8);
			this->half_y_ = this->declare_parameter<double>("map.field_half_y", 1.67);
			this->margin_ = this->declare_parameter<double>("map.margin", 0.2);
		}

		void run() {
			const auto points = this->collect_points();
			std::filesystem::create_directories(this->output_dir_);
			this->write_pcd(points);
			this->write_grid(points);
			RCLCPP_INFO(
				this->get_logger(),
				"%zu surfaces -> %zu points; wrote %s/%s.{pcd,pgm,yaml}",
				this->surfaces_.size(),
				points.size(),
				this->output_dir_.c_str(),
				this->name_.c_str()
			);
		}

	private:
		auto collect_points() const -> std::vector<Point2> {
			std::set<std::pair<std::int64_t, std::int64_t>> cells{};
			std::vector<Point2> points{};
			const auto add = [&](const double x, const double y) {
				const auto key = std::pair{
					static_cast<std::int64_t>(std::floor(x / this->point_spacing_)),
					static_cast<std::int64_t>(std::floor(y / this->point_spacing_)),
				};
				if (cells.insert(key).second) {
					points.push_back({(static_cast<double>(key.first) + 0.5) * this->point_spacing_,
									  (static_cast<double>(key.second) + 0.5) * this->point_spacing_});
				}
			};

			std::vector<Surface> moved{};
			std::vector<Point2> hits{};
			for (double vx = -this->half_x_; vx <= this->half_x_ + 1e-9; vx += this->viewpoint_step_) {
				for (double vy = -this->half_y_; vy <= this->half_y_ + 1e-9; vy += this->viewpoint_step_) {
					// 視点を原点に持ってくる
					const auto to_view = math::trans(Vec3{
						static_cast<float>(-vx), static_cast<float>(-vy), static_cast<float>(-this->height_)});
					moved = this->surfaces_;
					for (auto& s : moved) {
						std::visit([&](auto& v) { v.apply_se3(to_view); }, s);
					}

					hits.clear();
					bool inside = false;
					for (std::int64_t i = 0; i < this->ray_num_; ++i) {
						const double angle = 2.0 * std::numbers::pi * static_cast<double>(i)
							/ static_cast<double>(this->ray_num_);
						const UVec3 ray{static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)), 0.f};
						float nearest2 = std::numeric_limits<float>::infinity();
						for (const auto& s : moved) {
							std::visit([&](const auto& v) { nearest2 = std::min(nearest2, v.ray_collision(ray)); }, s);
						}
						const double d = std::sqrt(static_cast<double>(nearest2));
						if (!std::isfinite(d) || d > this->range_max_) { continue; }
						// 壁の中や、壁に接した視点は捨てる
						if (d < 0.05) {
							inside = true;
							break;
						}
						hits.push_back({vx + d * std::cos(angle), vy + d * std::sin(angle)});
					}
					if (inside) { continue; }
					for (const auto& h : hits) { add(h.x, h.y); }
				}
			}
			return points;
		}

		void write_pcd(const std::vector<Point2>& points) const {
			std::ofstream out{this->output_dir_ + "/" + this->name_ + ".pcd"};
			out << "# .PCD v0.7 - generated by oumuamua_sim make_field_map. do not edit by hand\n"
				<< "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
				<< "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
				<< "POINTS " << points.size() << "\nDATA ascii\n";
			out.setf(std::ios::fixed);
			out.precision(4);
			for (const auto& p : points) { out << p.x << ' ' << p.y << " 0\n"; }
		}

		void write_grid(const std::vector<Point2>& points) const {
			double min_x = std::numeric_limits<double>::infinity();
			double min_y = min_x;
			double max_x = -min_x;
			double max_y = -min_x;
			for (const auto& p : points) {
				min_x = std::min(min_x, p.x);
				min_y = std::min(min_y, p.y);
				max_x = std::max(max_x, p.x);
				max_y = std::max(max_y, p.y);
			}
			min_x -= this->margin_;
			min_y -= this->margin_;
			max_x += this->margin_;
			max_y += this->margin_;
			const double r = this->resolution_;
			const auto width = static_cast<std::int64_t>(std::ceil((max_x - min_x) / r));
			const auto height = static_cast<std::int64_t>(std::ceil((max_y - min_y) / r));

			// 254 = 空き、0 = 占有 (nav2_map_server の trinary、negate 0)
			std::vector<std::uint8_t> image(static_cast<std::size_t>(width * height), 254);
			for (const auto& p : points) {
				const auto col = static_cast<std::int64_t>(std::floor((p.x - min_x) / r));
				const auto row_from_bottom = static_cast<std::int64_t>(std::floor((p.y - min_y) / r));
				if (col < 0 || col >= width || row_from_bottom < 0 || row_from_bottom >= height) { continue; }
				// 画像の 1 行目が地図の上端 (y が大きい側)
				const auto row = height - 1 - row_from_bottom;
				image[static_cast<std::size_t>(row * width + col)] = 0;
			}

			{
				std::ofstream out{this->output_dir_ + "/" + this->name_ + ".pgm", std::ios::binary};
				out << "P5\n# generated by oumuamua_sim make_field_map\n" << width << ' ' << height << "\n255\n";
				out.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
			}
			std::ofstream yaml{this->output_dir_ + "/" + this->name_ + ".yaml"};
			yaml.setf(std::ios::fixed);
			yaml.precision(4);
			yaml << "# generated by oumuamua_sim make_field_map. do not edit by hand\n"
				 << "image: " << this->name_ << ".pgm\n"
				 << "mode: trinary\n"
				 << "resolution: " << r << "\n"
				 << "origin: [" << min_x << ", " << min_y << ", 0.0]\n"
				 << "negate: 0\n"
				 << "occupied_thresh: 0.65\n"
				 << "free_thresh: 0.25\n";
		}

		std::vector<Surface> surfaces_{};
		double height_{};
		std::string output_dir_{};
		std::string name_{};
		double resolution_{};
		double point_spacing_{};
		double viewpoint_step_{};
		std::int64_t ray_num_{};
		double range_max_{};
		double half_x_{};
		double half_y_{};
		double margin_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	{
		const auto node = std::make_shared<MakeFieldMap>();
		node->run();
	}
	rclcpp::shutdown();
	return 0;
}
