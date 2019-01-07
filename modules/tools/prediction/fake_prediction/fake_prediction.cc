/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/common/math/math_utils.h"


namespace apollo {
namespace prediction {

/**
 * class FakePredictionComponent
 * This class generates fake prediction messages. The prediction message only
 * has valid headers.
 *
 * This tool is used to trigger modules that depends on prediction message.
 */
using ::apollo::common::TrajectoryPoint;
using ::apollo::perception::PerceptionObstacles;
using apollo::common::math::Vec2d;

typedef struct {double x; double y;} Point2d;

class FakePredictionComponent : public apollo::cyber::TimerComponent {
 public:
  bool Init() override {
    prediction_writer_ =
        node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);
    perception_writer_ = node_->CreateWriter<PerceptionObstacles>(FLAGS_perception_obstacle_topic);
    return true;
  }
  bool Proc() override {
    auto prediction = std::make_shared<PredictionObstacles>();
    common::util::FillHeader("fake_prediction", prediction.get());
    auto prediction_obstacles = prediction.get();

    const double delta_ts = 0.1;
    const double x = 586135;
    const double y = 4140056;
    const double heading = -1.78;
    const double total_time = 8.0;
    const double speed = 0.0;

    TrajectoryPoint point;
    point.mutable_path_point()->set_x(x);
    point.mutable_path_point()->set_y(y);
    point.mutable_path_point()->set_theta(heading);
    point.set_v(speed);
    point.set_a(0.0);

    Trajectory trajectory;
    for (int i = 0; i < static_cast<int>(total_time / delta_ts); ++i) {
      point.set_relative_time(i * delta_ts);
      trajectory.add_trajectory_point()->CopyFrom(point);
    }
    auto prediction_obs_ptr = prediction_obstacles->add_prediction_obstacle();
    prediction_obs_ptr->add_trajectory()->CopyFrom(trajectory);
    prediction_obs_ptr->set_is_static(true);
    prediction_obs_ptr->set_predicted_period(total_time);

    /* Fake perception obstacle for visualization */
    auto perception = std::make_shared<PerceptionObstacles>();
    common::util::FillHeader("fake_prediction", perception.get());
    auto perception_obstacles = perception.get();

    std::vector<Vec2d> corners;
    corners.emplace_back(2.5, -1);
    corners.emplace_back(2.5, 1);
    corners.emplace_back(-2.5, 1);
    corners.emplace_back(-2.5, -1);

    double max_x = std::numeric_limits<double>::min();
    double max_y = std::numeric_limits<double>::min();
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();

    for (const auto& pt : corners) {
      max_x = std::max(max_x, pt.x());
      min_x = std::min(min_x, pt.x());
      max_y = std::max(max_y, pt.y());
      min_y = std::min(min_y, pt.y());
    }

    const double length = max_x - min_x;
    const double width = max_y - min_y;
    const double height = 1.5;
    auto obs_ptr = perception_obstacles->add_perception_obstacle();
    auto obj_center = obs_ptr->mutable_position();
    obj_center->set_x(x);
    obj_center->set_y(y);
    obj_center->set_z(height/2.0);
    obs_ptr->set_theta(heading);

    auto obj_velocity = obs_ptr->mutable_velocity();
    obj_velocity->set_x(0);
    obj_velocity->set_y(0);
    obj_velocity->set_z(0);

    obs_ptr->set_length(length);
    obs_ptr->set_width(width);
    obs_ptr->set_height(height);

    Vec2d center_point(x, y);
    std::vector<Vec2d> points;
    points.reserve(corners.size());
    for (const auto& relative_pt : corners) {
      points.push_back(
          Vec2d(relative_pt.x(), relative_pt.y()).rotate(heading));
      points.back() += center_point;
    }

      for (const auto& pt : points) {
        auto point_ptr = obs_ptr->add_polygon_point();
        point_ptr->set_x(pt.x());
        point_ptr->set_y(pt.y());
        point_ptr->set_z(height);
      }
      obs_ptr->set_type(::apollo::perception::PerceptionObstacle::Type::PerceptionObstacle_Type_VEHICLE);

      prediction_obs_ptr->mutable_perception_obstacle()->CopyFrom(*obs_ptr);

      prediction_writer_->Write(prediction);
      perception_writer_->Write(perception);
      return true;
  }

 private:

  std::shared_ptr<apollo::cyber::Writer<PredictionObstacles>>
      prediction_writer_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> perception_writer_;
};
CYBER_REGISTER_COMPONENT(FakePredictionComponent);

}  // namespace prediction
}  // namespace apollo
