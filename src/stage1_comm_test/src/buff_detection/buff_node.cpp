#include "big_rune_gimbal.hpp"
#include "big_rune_trajectory.hpp"
#include "big_rune_observation.hpp"
#include "buff_detection/buff_visual.hpp"
#include "buff_detection/buff_motion.hpp"
#include "buff_detection/big_rune_cycle.hpp"
#include "aiming/rune_gimbal_feedback.hpp"
#include "buff_detection/buff_session.hpp"
#include "common/ros_aim_utils.hpp"
#include <campus_aim_interfaces/msg/buff_solution.hpp>
#include <campus_aim_interfaces/msg/buff_status.hpp>
#include <tdt_interface/msg/rune_request.hpp>
#include <tdt_interface/msg/rune_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <map>
#include <csignal>
#include <thread>

namespace {
using State = tdt_interface::msg::AutoAimState;
using Solution = campus_aim_interfaces::msg::BuffSolution;
using Status = campus_aim_interfaces::msg::BuffStatus;
using Request = tdt_interface::msg::RuneRequest;

double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::map<std::string, std::string> fields(const std::string& value) {
    std::map<std::string, std::string> f;
    std::istringstream s(value);
    s >> f["stage"];
    std::string t;
    while (s >> t) {
        auto n = t.find('=');
        if (n != std::string::npos)
            f[t.substr(0, n)] = t.substr(n + 1);
    }

    return f;
}

uint64_t id(const std::map<std::string, std::string>& f, const std::string& key) {
    try {
        return std::stoull(f.at(key));
    } catch (...) {
        return 0;
    }
}
}

class BuffNode final : public rclcpp::Node {
public:
    BuffNode() : Node("buff_controller"), detector_(false) {
        int player = declare_parameter<int>("player_id", 1);
        int limit=declare_parameter<int>("max_opportunities",5);
        if(limit<1 || limit>5) throw std::invalid_argument("max_opportunities must be 1..5");
        session_.setLimit(limit);
        if (player != 1 && player != 2)
            throw std::invalid_argument("Invalid buff player_id");
        plane_ = campus_buff::RunePlane::forPlayer(player);
        search_center_ = plane_.center;
        detector_.setRed(player == 2);
        display_ = declare_parameter<bool>("display", false);

        nominal_speed_ = declare_parameter<double>("default_projectile_speed_mps", 23.5);
        if (!std::isfinite(nominal_speed_) || nominal_speed_ < 10 || nominal_speed_ > 35)
            throw std::invalid_argument("Invalid nominal rune projectile speed");
        latency_ = declare_parameter<double>("command_latency_s", .04);
        physical_radius_ = declare_parameter<double>("hub_to_leaf_radius_m", .700);
        if (!std::isfinite(physical_radius_) || physical_radius_ < .5 || physical_radius_ > .9)
            throw std::invalid_argument("Invalid physical rune radius");
        if (!std::isfinite(latency_) || latency_ < 0 || latency_ > .1)
            throw std::invalid_argument("Invalid buff command latency");

        cv::setNumThreads(2);

        auto qos = campus::latestQoS();
        auto cached = rclcpp::QoS(1).reliable().transient_local();

        solution_pub_ = create_publisher<Solution>("/buff/solution", qos);
        status_pub_ = create_publisher<Status>("/buff/status", qos);
        request_pub_ = create_publisher<Request>("/game/rune/request", qos);

        shot_sub_ = create_subscription<std_msgs::msg::UInt64>(
            "/buff/shot_sent", qos, [this](std_msgs::msg::UInt64::ConstSharedPtr m) {
                auto it = offers_.find(m->data);
                if (it == offers_.end())
                    return;
                const auto offer = it->second;
                offers_.erase(it);
                if (offer.generation != leaves_.generation()) return;
                bool miss_rescue=leaves_.rescueReady(),miss_retry=leaves_.retryReady(offer.slot);
                leaves_.sent(offer.slot, now().seconds() + offer.flight + latency_ + .10);
                shot_ammo_ = offer.ammo;
                shot_stamp_ = now().seconds();
                shot_flight_=offer.flight;shot_no_ammo_stamp_=shot_stamp_-.015;
                shot_time_ = steady();
                shot_slot_ = offer.slot;
                shot_confirmed_ = false;
                extra_shot_noted_ = false;
                next_shot_ = std::max(next_shot_, steady() + offer.flight + latency_ + (session_.big()?.06:.10));
                RCLCPP_INFO(get_logger(), "RUNE_SHOT_SENT id=%lu leaf=%d wait_flight=%.3f", m->data, offer.slot,
                            offer.flight);
                if(miss_retry)RCLCPP_INFO(get_logger(),"BIG_MISS_RETRY leaf=%d: consumed shot still pending in fresh post-impact images",offer.slot);
                else if(miss_rescue)RCLCPP_INFO(get_logger(),"BIG_MISS_RESCUE leaf=%d: unshot plate in current pair",offer.slot);
                if (m->data == issued_shot_)
                    issued_shot_ = 0;
            });

        state_sub_ = create_subscription<State>("/autoaim/state", qos, [this](State::ConstSharedPtr s) {
            if (state_ && campus::stampSeconds(s->header.stamp) <= campus::stampSeconds(state_->header.stamp))
                return;
            if (state_ &&
                (s->frame_sequence < state_->frame_sequence || s->header.frame_id != state_->header.frame_id ||
                 campus::sessionKey(*s) != campus::sessionKey(*state_) ||
                 s->remaining_seconds > state_->remaining_seconds + 2.)) {
                feedback_.reset();
                session_.reset();
                goal_ = 0;
                active_ = false;
                buff_active_ = false;
                feedback_known_ = false;
                last_accepted_id_ = 0;
                cooldown_wait_ = false;
                pending_until_ = 0;
                guard_.clear();
                arrived_=false;follow_goal_=0;follow_time_=-INFINITY;
                request_=Request();request_stamp_=0;
                match_start_stamp_=campus::stampSeconds(s->header.stamp);
                resetVision();
                mission_time_ = -INFINITY;
                RCLCPP_INFO(get_logger(),"RUNE_MATCH_RESET sequence=%lu",s->frame_sequence);
            }
            feedback_.update(campus::stampSeconds(s->header.stamp), s->yaw_degrees, s->pitch_degrees);
            big_feedback_.measured(campus::stampSeconds(s->header.stamp), s->yaw_degrees, s->pitch_degrees);
            state_ = s;
            state_time_ = steady();
        });

        mission_sub_ = create_subscription<std_msgs::msg::String>(
            "/mission/status", cached, [this](std_msgs::msg::String::ConstSharedPtr m) {
                auto f = fields(m->data);
                auto goal = id(f, "goal");
                bool active = f["stage"] == "BUFF";

                if (active && (!active_ || (session_.finished() && goal != goal_))) {
                    session_.begin(goal, steady());
                    resetVision();
                } else if (!active && active_) {
                    session_.abort("MISSION_LEFT");
                    resetVision();
                }
                active_ = active;
                goal_ = goal;
                mission_time_ = steady();
            });

        follow_sub_ = create_subscription<std_msgs::msg::String>("/navigation/follow_status", cached,
                                                                 [this](std_msgs::msg::String::ConstSharedPtr m) {
                                                                     auto f = fields(m->data);
                                                                     arrived_ = f["stage"] == "ARRIVED";
                                                                     follow_goal_ = id(f, "goal");
                                                                     follow_time_ = steady();
                                                                 });

        cancel_sub_ = create_subscription<std_msgs::msg::Bool>("/planning/cancel", qos,
                                                               [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                                                                   if (m->data) {
                                                                       session_.abort("CANCELLED");
                                                                       resetVision();
                                                                   }
                                                               });

        result_sub_ = create_subscription<tdt_interface::msg::RuneResult>(
            "/game/rune/result", qos, [this](tdt_interface::msg::RuneResult::ConstSharedPtr m) {
                double age = now().seconds() - campus::stampSeconds(m->header.stamp);
                // An authoritative matching result does not depend on camera cadence.
                if (age < -.1 || age > .3 ||
                    campus::stampSeconds(m->header.stamp) < request_stamp_)
                    return;
                // Effect expiry is reported while we are fighting robots too.
                // Only our own accepted request can update this lifecycle.
                if (m->request_id && m->request_id == last_accepted_id_ && m->accepted) {
                    buff_active_ = m->activated;
                    feedback_known_ = true;
                }

                if (active_ && m->request_id == session_.request()) {
                    session_.result(m->request_id, m->accepted, m->activated, steady());
                    if (m->accepted) {
                        if (m->request_id != last_accepted_id_) {
                            pending_until_ = session_.sent() + 20.;
                            if(session_.big())leaves_.beginBig(request_stamp_);
                        }
                        if (m->activated)
                            pending_until_ = 0;
                        last_accepted_id_ = m->request_id;
                        buff_active_ = m->activated;
                        feedback_known_ = true;
                    }
                }
            });

        frame_sub_ = create_subscription<campus::Frame>("/autoaim/frame", qos,
                                                        [this](campus::Frame::ConstSharedPtr f) { process(*f); });

        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
    }

    void stop() {
        Solution s;
        s.header.stamp = now();
        s.reason = "STOPPED";

        solution_pub_->publish(s);
    }

private:
    bool transportFresh() const {
        if (!state_)
            return false;
        double age = now().seconds() - campus::stampSeconds(state_->header.stamp);
        return steady() - state_time_ <= .28 && age >= -.1 && age <= .28 && state_->remaining_seconds > 0 &&
               std::isfinite(state_->remaining_seconds);
    }

    bool freshState() const {
        return transportFresh() && state_->health > 0;
    }

    bool positionReady() const {
        return arrived_ && follow_goal_ == goal_ && steady() - follow_time_ < .2 &&
               std::hypot(state_->twist.linear.x, state_->twist.linear.y) / 3. < .03;
    }

    void resetVision(bool new_visit = true) {
        big_feedback_.reset();
        last_measured_hub_ = -INFINITY;
        last_rotor_stamp_ = -INFINITY;
        last_pending_stamp_ = -INFINITY;
        last_frame_stamp_ = -1;
        vision_reason_ = "WAIT_RUNE_FRAME";
        motion_.reset();
        detector_.reset();
        observation_ = State();
        have_observation_ = false;
        selected_ = -1;
        pending_current_ = false;
        last_aim_time_ = -INFINITY;
        have_hub_ = false;
        detector_.setHubSearch({});
        if (new_visit) {
            preferred_generation_ = UINT64_MAX;
            leaves_.reset();
            offers_.clear();
            next_shot_ = 0;
            issued_shot_ = 0;
            shot_slot_ = -1;
            shot_confirmed_ = false;
            extra_shot_noted_ = false;
            shot_flight_=0;shot_no_ammo_stamp_=-INFINITY;
        }
    }

    void process(const campus::Frame& frame) {
        if (!active_ || !session_.accepted() || session_.finished() || !freshState() || !positionReady())
            return;
        pending_current_ = false;
        detected_pending_ = solved_leaves_ = consistent_leaves_ = 0;
        bool reset = false;

        // Reject another match's queued frame BEFORE it can mutate the guard
        // and reset a newly acquired rotor in this match.
        if(campus::stampSeconds(frame.header.stamp)<match_start_stamp_ ||
           frame.state.header.frame_id!=state_->header.frame_id ||
           campus::sessionKey(frame.state)!=campus::sessionKey(*state_) ||
           frame.state.frame_sequence>state_->frame_sequence+5)return;
        if (!guard_.check(frame, now().seconds(), .28, reset) || !campus::validCamera(frame)) {
            if (reset)
                resetVision(false);
            return;
        }

        if (reset)
            resetVision(false);

        cv::Mat image;
        try {
            image = campus::decodeFrame(frame);
        } catch (const cv::Exception&) {
            have_observation_ = false;
            return;
        }

        if (image.empty() || image.cols != int(frame.camera_info.width) ||
            image.rows != int(frame.camera_info.height)) {
            have_observation_ = false;
            return;
        }

        double stamp = campus::stampSeconds(frame.header.stamp);
        if (session_.big() && stamp < request_stamp_)
            return; // Queued images from a previous opportunity cannot seed this big model.
        auto camera = campus::cameraGeometry(frame, 3., {0, 0, .08});
        auto k = campus::intrinsics(frame);
        auto d = campus::distortion(frame);
        if (!d.empty() && cv::norm(d) > 0) {
            cv::Mat undistorted;
            cv::undistort(image, undistorted, k, d);
            image = undistorted;
        }
        d = cv::Mat();
        auto rectification = plane_.rectification(camera, k), inverse_rectification = rectification.inv();
        cv::Mat frontal;
        cv::warpPerspective(image, frontal, rectification, image.size());
        cv::Point2f expected;
        bool projected_visual_hub=false;
        // Restrict R only after a real visual hub has been solved. The coarse
        // map landmark never substitutes for a measurement or supplies range.
        if (have_hub_ && camera.project(hub_, k, d, expected)) {
            expected = plane_.mapPixel(rectification, expected);
            // Continue from a measured stationary hub while real leaves keep
            // validating the rotor geometry; a lost R glyph must not stop small runes.
            projected_visual_hub=(stamp-last_measured_hub_<=1.2 || stamp-last_rotor_stamp_<.3) &&
                std::isfinite(expected.x) && std::isfinite(expected.y) && expected.x>=6 && expected.y>=6 &&
                expected.x<image.cols-6 && expected.y<image.rows-6;
            double depth = camera.toCamera(hub_)[2];
            int radius = std::clamp(int(k(0, 0) * .12 / std::max(1., depth)), 18, 100);
            detector_.setHubSearch(
                cv::Rect(int(expected.x) - radius, int(expected.y) - radius, 2 * radius + 1, 2 * radius + 1));
        } else
            detector_.setHubSearch({});

        cv::Mat annotated = detector_.process(
            frontal, last_frame_stamp_ > 0 && stamp > last_frame_stamp_ ? 1. / (stamp - last_frame_stamp_) : 0, stamp,
            projected_visual_hub?&expected:nullptr);
        last_frame_stamp_ = stamp;
        const auto& r = detector_.centerResult();
        const cv::Point2f hub_pixel=r.observed?r.center:expected;
        if (session_.big() && (r.observed || r.held || projected_visual_hub) && detector_.ringResults().empty())
            leaves_.observeDark(stamp);
        vision_reason_ = r.observed ? "WAIT_PENDING_LEAF" : "SEARCH_R";
        if (r.observed || projected_visual_hub) {
            struct Leaf {
                double phase;
                cv::Point2f pixel;
                bool pending;
                cv::Vec3d hub, target;
            };
            std::vector<Leaf> leaves;

            for (const auto& ring : detector_.ringResults()) {
                detected_pending_ += ring.pending;
                double ratio = cv::norm(ring.center - hub_pixel) / ring.radius;
                if (ratio < 3. || ratio > 9.)
                    continue;
                cv::Vec3d hub, target;
                auto r_pixel = plane_.mapPixel(inverse_rectification, hub_pixel),
                     leaf_pixel = plane_.mapPixel(inverse_rectification, ring.center);
                if (std::isfinite(r_pixel.x) && std::isfinite(leaf_pixel.x) &&
                    plane_.solveHub(r_pixel, leaf_pixel, camera, k, d, physical_radius_, hub, target))
                    leaves.push_back({plane_.phase(target - hub), leaf_pixel, ring.pending, hub, target});
            }

            solved_leaves_ = leaves.size();
            if (!leaves.empty()) {
                // The stationary rotor centre is persistent geometry. An
                // outlier from one completed ring must not reset target,
                // motion fit and shot acknowledgement together.
                std::sort(leaves.begin(), leaves.end(), [&](const auto& a, const auto& b) {
                    return cv::norm(a.hub - camera.origin) < cv::norm(b.hub - camera.origin);
                });
                auto measured = leaves[leaves.size() / 2].hub;
                bool measured_pending=true;
                if(session_.big()) {
                    std::vector<cv::Vec3d> pending_hubs;
                    for(const auto& leaf:leaves)if(leaf.pending)pending_hubs.push_back(leaf.hub);
                    measured_pending=!pending_hubs.empty();
                    if(measured_pending)measured=campus_buff::bigRuneObservedHub(pending_hubs);
                }
                if (!have_hub_) {
                    hub_ = measured;
                    have_hub_ = true;
                } else if (measured_pending && cv::norm(measured - hub_) < .12)
                    hub_ = .98 * hub_ + .02 * measured;
                plane_.center = hub_;
                std::vector<Leaf> consistent, pending;

                for (auto leaf : leaves) {
                    cv::Vec3d target;
                    const auto r_pixel = plane_.mapPixel(inverse_rectification, hub_pixel);
                    if (!plane_.trackedLeaf(r_pixel, leaf.pixel, camera, k, d, physical_radius_, target))
                        continue;
                    leaf.hub = hub_;
                    leaf.target = target;
                    leaf.phase = plane_.phase(target - hub_);
                    consistent.push_back(leaf);
                    if (leaf.pending)
                        pending.push_back(leaf);
                }

                consistent_leaves_ = consistent.size();
                if(r.observed && !consistent.empty())last_measured_hub_=stamp;
                if (!consistent.empty()) {
                    last_rotor_stamp_ = stamp;
                    std::vector<campus_buff::RuneLeafObservation> observed;

                    for (const auto& leaf : consistent)
                        observed.push_back({leaf.phase, leaf.pending});
                    // Absence of completed rings proves rollback only when the
                    // whole physical rotor is inside both source and rectified views.
                    bool full_view=true;
                    for(int i=0;i<32 && full_view;++i) {
                        double a=i*2*CV_PI/32.;
                        auto world=hub_+plane_.right*((physical_radius_+.12)*std::cos(a))+
                            cv::Vec3d(0,0,(physical_radius_+.12)*std::sin(a));
                        cv::Point2f pixel;
                        full_view=camera.project(world,k,d,pixel);
                        auto frontal_pixel=plane_.mapPixel(rectification,pixel);
                        for(auto p:{pixel,frontal_pixel})
                            full_view=full_view && std::isfinite(p.x) && std::isfinite(p.y) &&
                                p.x>=6 && p.y>=6 && p.x<image.cols-6 && p.y<image.rows-6;
                    }
                    if(motion_.readyToFire())
                        leaves_.predictRotation(motion_.stamp(),stamp,motion_.advance(stamp-motion_.stamp()),session_.big());
                    // Reject an outlier image before it changes physical leaf
                    // identity or reuses a previously valid model to authorize fire.
                    const auto& reference = pending.empty() ? consistent.front() : pending.front();
                    double observed_phase=reference.phase;
                    if(session_.big() && !pending.empty()) {
                        std::vector<double> phases;
                        for(const auto& leaf:pending)phases.push_back(leaf.phase);
                        observed_phase=campus_buff::bigRuneObservedPhase(phases);
                    }
                    const bool use_motion_sample=!session_.big() || !pending.empty();
                    if(use_motion_sample && !motion_.update(observed_phase, stamp, session_.big())) {
                        have_observation_=false;
                        pending_current_=false;
                        vision_reason_="REJECT_RUNE_PHASE_OUTLIER";
                        return;
                    }
                    auto generation=leaves_.generation();
                    leaves_.observe(observed, stamp, session_.big(),full_view);
                    if(generation!=leaves_.generation()) {
                        offers_.clear();issued_shot_=0;shot_slot_=-1;shot_confirmed_=false;extra_shot_noted_=false;
                        selected_=-1;target_change_stamp_=now().seconds();preferred_generation_=UINT64_MAX;
                        RCLCPP_INFO(get_logger(),"RUNE_SEQUENCE_RESTART generation=%lu",leaves_.generation());
                    }
                    // Track rotation from a real visible leaf; modulo-72-degree
                    // association in RotorMotion retains the model at a switch.
                    {
                        // For a new big-rune pair, acquire the nearer actual
                        // plate first. Contour/depth order is not a tactical
                        // priority and may demand an unnecessary full slew.
                        if (session_.big() && pending.size() == 2) {
                            auto barrel=feedback_.predict(now().seconds(),latency_);
                            double yaw=barrel.valid?barrel.yaw:state_->yaw_degrees;
                            double pitch=barrel.valid?barrel.pitch:state_->pitch_degrees;
                            auto cost=[&](const auto& leaf) {
                                auto b=campus::solveBallistics(leaf.target-camera.origin,
                                    campus_buff::runeProjectileSpeed(state_->projectile_speed,nominal_speed_));
                                if (motion_.readyToFire()) {
                                    cv::Vec3d pivot{state_->position.x/3.,state_->position.y/3.,state_->position.z/3.};
                                    b=campus_buff::intercept(plane_,hub_,leaf.phase,motion_,pivot,yaw,pitch,
                                        campus_buff::runeProjectileSpeed(state_->projectile_speed,nominal_speed_),
                                        std::max(0.,now().seconds()-stamp),latency_,physical_radius_).ballistic;
                                }
                                return b.valid?std::hypot(std::remainder(b.yaw-yaw,360.),b.pitch-pitch):INFINITY;
                            };
                            if(cost(pending[1])<cost(pending[0])) std::swap(pending[0],pending[1]);
                        }
                        std::vector<int> pending_slots;
                        for (const auto& leaf : pending) pending_slots.push_back(leaves_.slot(leaf.phase));
                        int next_slot = leaves_.selectPending(pending_slots, selected_, session_.big());
                        // Keep the already acquired pending plate when the sine
                        // fit becomes ready; switching here restarts settling at
                        // the end of the first 2.5 s window.
                        auto best = std::find_if(pending.begin(), pending.end(), [&](const auto& leaf) {
                            return next_slot >= 0 && leaves_.slot(leaf.phase) == next_slot;
                        });
                        if(session_.big() && best==pending.end() && !pending.empty()) {
                            bool available=false;
                            for(int i=0;i<5;++i)available=available||leaves_.available(i);
                            if(!available) {
                                // Waiting for a bullet to arrive is NOT target
                                // loss. Keep tracking a genuinely pending leaf
                                // without fire, so a measured miss can be
                                // retried without another acquisition slew.
                                best=std::find_if(pending.begin(),pending.end(),[&](const auto& leaf) {
                                    return leaves_.slot(leaf.phase)==selected_;
                                });
                                if(best==pending.end())best=pending.begin();
                            }
                        }

                        if (best != pending.end()) {
                            int slot = leaves_.slot(best->phase);
                            if (issued_shot_ && (issued_slot_ != slot || issued_generation_ != leaves_.generation()))
                                issued_shot_ = 0;
                            if (slot != selected_) {
                                target_change_stamp_ = now().seconds();
                                RCLCPP_INFO(get_logger(), "RUNE_LEAF_SWITCH %d -> %d", selected_, slot);
                            }
                            selected_ = slot;
                            phase_ = best->phase;
                            target_radius_ = physical_radius_;
                            observation_ = frame.state;
                            have_observation_ = true;
                            pending_current_ = true;
                            last_pending_stamp_ = stamp;
                            cv::drawMarker(annotated, plane_.mapPixel(rectification, best->pixel), {0, 255, 255},
                                           cv::MARKER_CROSS, 24, 2);
                        }
                    }

                    if (!pending_current_ && have_observation_ && leaves_.available(selected_) &&
                        stamp - last_pending_stamp_ < .12) {
                        // An unconfirmed one-frame detection gap is different
                        // from a completed leaf. Bridge it without permission
                        // to fire, rather than jerk back to R and lose settling.
                        phase_ += std::remainder(reference.phase - phase_, 2 * CV_PI / 5);
                        observation_ = frame.state;
                    } else if (!pending_current_) {
                        // Stop tracking the completed leaf, but retain the last
                        // command direction while searching. Returning to R here
                        // creates a needless slew before every new light.
                        have_observation_ = false;
                        selected_ = -1;
                        vision_reason_ = pending.empty() ? "SEARCH_NEXT_PENDING" : "WAIT_LEAF_CHANGE";
                    }
                } else if (stamp - last_rotor_stamp_ > 1.) {
                    have_hub_ = false;
                    detector_.setHubSearch({});
                }
            }
        } else if (have_observation_ && stamp - campus::stampSeconds(observation_.header.stamp) > .4) {
            // Reopen the image search if a geometric prior could no longer be verified.
            have_hub_ = false;
            detector_.setHubSearch({});
        }

        if (display_) {
            cv::putText(annotated, "Fire: " + last_reason_, {12, 98}, cv::FONT_HERSHEY_SIMPLEX, .55, {0, 255, 255}, 1);
            cv::putText(annotated, diagnostic_, {12, 121}, cv::FONT_HERSHEY_SIMPLEX, .45, {0, 255, 255}, 1);
            cv::imshow("competition_buff", annotated);
            cv::waitKey(1);
        }
    }

    void tick() {
        double alignment = NAN, raw_alignment = NAN;
        double time = steady();
        Solution out;
        out.header.stamp = now();
        out.goal_id = goal_;
        out.request_id = session_.request();
        out.command_latency = latency_;
        out.big_rune = session_.big();
        out.reason = "INACTIVE";
        if (state_) {
            out.header.frame_id = state_->header.frame_id;
            out.state_session = campus::sessionKey(*state_);
            out.frame_sequence = state_->frame_sequence;
        }

        bool ready = active_ && steady() - mission_time_ < .5 && freshState();
        if (ready) {
            // A command acknowledgement is not proof that Unity fired. Re-arm
            // only if a later measured state proves no ammunition was consumed.
            // A consumed round waits for post-flight visual confirmation.
            if (shot_slot_ >= 0) {
                if (state_->ammo_remaining < shot_ammo_ &&
                    campus::stampSeconds(state_->header.stamp) >= shot_stamp_) {
                    if(!shot_confirmed_) {
                        shot_confirmed_ = true;
                        leaves_.confirmConsumed(shot_slot_,campus::stampSeconds(state_->header.stamp)+shot_flight_,
                            shot_no_ammo_stamp_+shot_flight_);
                        if(session_.big())RCLCPP_INFO(get_logger(),
                            "RUNE_AMMO_CONFIRMED leaf=%d release_delay_interval=[%.3f,%.3f] flight=%.3f",
                            shot_slot_,shot_no_ammo_stamp_-shot_stamp_,campus::stampSeconds(state_->header.stamp)-shot_stamp_,shot_flight_);
                    }
                    if(!extra_shot_noted_ && state_->ammo_remaining<=shot_ammo_-2) {
                        leaves_.extraConsumed(shot_slot_);
                        extra_shot_noted_=true;
                        RCLCPP_WARN(get_logger(),"RUNE_EXTRA_AMMO leaf=%d: wait for visual sequence reset",shot_slot_);
                    }
                }
                else if (!shot_confirmed_ && state_->ammo_remaining == shot_ammo_) {
                    shot_no_ammo_stamp_=std::max(shot_no_ammo_stamp_,campus::stampSeconds(state_->header.stamp));
                    if(campus::stampSeconds(state_->header.stamp)>shot_stamp_+(session_.big()?.20:.30)) {
                        leaves_.notFired(shot_slot_);
                        shot_slot_ = -1;
                    }
                }
            }

            bool stopped = positionReady();

            if (buff_active_ && !session_.request())
                cooldown_wait_ = true;
            else {
                if (cooldown_wait_) {
                    session_.begin(goal_, time);
                    resetVision();
                    cooldown_wait_ = false;
                }

                if (stopped)
                    session_.arrived(time);
                session_.tick(time);
            }

            if (state_->is_weakened || state_->ammo_remaining <= 0)
                session_.abort("REFILL_REQUIRED");
            if (count_publishers(request_pub_->get_topic_name()) > 1)
                session_.abort("MULTIPLE_REQUEST_PUBLISHERS");
            if (!session_.request() && !session_.canRequest() && !session_.finished())
                session_.abort("NO_KNOWN_OPPORTUNITY");

            if (!session_.finished() && stopped && !buff_active_ && time >= pending_until_ && session_.canRequest()) {
                uint64_t request = std::max(last_id_ + 1, static_cast<uint64_t>(now().nanoseconds()));
                last_id_ = request;
                session_.requested(request, time);
                request_.header.stamp = now();
                request_.request_id = request;
                request_stamp_ = campus::stampSeconds(request_.header.stamp);
                request_pub_->publish(request_);
                last_retry_ = time;
                resetVision();
            }

            if (!session_.finished() && session_.request() && !session_.accepted() && time - last_retry_ > .2) {
                // Retry the SAME id and original timestamp, never consume another opportunity.
                request_pub_->publish(request_);
                last_retry_ = time;
            }
            out.request_id = session_.request();
            out.reason =
                buff_active_ && !session_.request()
                    ? "WAIT_BUFF_EFFECT_END"
                    : (time < pending_until_ && !session_.request() ? "WAIT_PREVIOUS_ACTIVATION" : session_.reason());
            if (stopped && !session_.finished()) {
                cv::Vec3d pivot{state_->position.x / 3., state_->position.y / 3., state_->position.z / 3.};
                cv::Vec3d acquire = have_hub_ ? hub_ : search_center_;
                if (have_hub_ && have_observation_ &&
                    now().seconds() - campus::stampSeconds(observation_.header.stamp) <= .28)
                    acquire = plane_.target(hub_, phase_, target_radius_);
                auto look = campus::solveBallistics(
                    acquire - pivot, campus_buff::runeProjectileSpeed(state_->projectile_speed, nominal_speed_));
                out.yaw = look.yaw;
                out.pitch = look.pitch;
                out.valid = look.valid;
                // A missing target must not snap the gimbal back to R every
                // other frame. Hold the last aim without fire across brief gaps.
                if (session_.accepted() && std::isfinite(last_aim_time_) && !leaves_.needsRestartView()) {
                    out.yaw = last_aim_yaw_;
                    out.pitch = last_aim_pitch_;
                    out.valid = true;
                }

                if (session_.accepted() && !have_observation_)
                    out.reason = leaves_.needsRestartView() ? "CONFIRM_RUNE_SEQUENCE_RESET" : vision_reason_;
                if (session_.accepted() && have_observation_) {
                    double age = now().seconds() - campus::stampSeconds(observation_.header.stamp);
                    out.observation_stamp = observation_.header.stamp;
                    out.reason = "WAIT_MOTION";
                    if (age >= 0 && age <= .28 && pending_current_) {
                        auto measured = campus::solveBallistics(
                            plane_.target(hub_, phase_, target_radius_) - pivot,
                            campus_buff::runeProjectileSpeed(state_->projectile_speed, nominal_speed_));
                        if (measured.valid) {
                            out.yaw = measured.yaw;
                            out.pitch = measured.pitch;
                            out.valid = true;
                            out.reason = "ACQUIRE_PENDING_NO_FIRE";
                        }
                    }
                    if (age >= 0 && age <= .28 && motion_.ready()) {
                        if(session_.big()) {
                            out.header.stamp=now();
                            age=campus::stampSeconds(out.header.stamp)-campus::stampSeconds(observation_.header.stamp);
                        }
                        double speed = campus_buff::runeProjectileSpeed(state_->projectile_speed, nominal_speed_);
                        auto aim = campus_buff::intercept(plane_, hub_, phase_, motion_, pivot, state_->yaw_degrees,
                                                          state_->pitch_degrees, speed, age, latency_, target_radius_);
                        out.valid = aim.ballistic.valid;
                        out.yaw = aim.ballistic.yaw;
                        out.pitch = aim.ballistic.pitch;
                        out.horizon = aim.horizon;
                        if(session_.big() && out.valid && age>=0 && age<=.27) {
                            const bool forward=age<.01;
                            auto before=campus_buff::intercept(plane_,hub_,phase_,motion_,pivot,state_->yaw_degrees,
                                state_->pitch_degrees,speed,forward?age+.01:age-.01,latency_,target_radius_);
                            auto after=campus_buff::intercept(plane_,hub_,phase_,motion_,pivot,state_->yaw_degrees,
                                state_->pitch_degrees,speed,forward?age+.02:age+.01,latency_,target_radius_);
                            if(before.ballistic.valid && after.ballistic.valid) {
                                auto path=(forward?campus_buff::RuneAimTrajectory::fitForward:campus_buff::RuneAimTrajectory::fit)(
                                    out.yaw,out.pitch,before.ballistic.yaw,before.ballistic.pitch,
                                    after.ballistic.yaw,after.ballistic.pitch,.01);
                                out.trajectory_valid=path.valid;
                                out.yaw_rate=path.yaw_rate;out.pitch_rate=path.pitch_rate;
                                out.yaw_acceleration=path.yaw_acceleration;out.pitch_acceleration=path.pitch_acceleration;
                            }
                        }
                        out.reason = out.valid ? "TRACK_PENDING" : "NO_RUNE_INTERCEPT";

                        if (out.valid) {
                            last_aim_yaw_ = out.yaw;
                            last_aim_pitch_ = out.pitch;
                            last_aim_time_ = time;
                            // A short linear feedback fit can look aligned during a
                            // post-switch slew. Require actual measured tracking.
                            auto barrel=big_feedback_.fireEstimate(now().seconds(),latency_,out.yaw,out.pitch,session_.big());
                            raw_alignment = std::hypot(std::remainder(out.yaw - state_->yaw_degrees, 360.),
                                                       out.pitch - state_->pitch_degrees);
                            double error = barrel.valid ? std::hypot(std::remainder(out.yaw - barrel.yaw, 360.),
                                                                     out.pitch - barrel.pitch)
                                                        : INFINITY;
                            alignment = error;
                            // Delayed feedback from BEFORE a leaf switch can
                            // extrapolate neatly to the new angle while the
                            // real barrel is still slewing. Require post-switch
                            // measured history, not only a small predicted error.
                            // The short feedback fallback needs .08-.10 s of
                            // post-switch samples. Longer polynomial fits still
                            // validate residuals, withheld samples and slew bounds.
                            bool target_feedback =
                                campus::stampSeconds(state_->header.stamp) >= target_change_stamp_ + .08;
                            bool impact_in_round = leaves_.impactInRound(
                                now().seconds() + aim.ballistic.flight_time + latency_, session_.big());
                            // Big-rune scores/collapse require the plate centre.
                            // Keep the established small-rune gates unchanged.
                            const double fire_alignment = session_.big() ? .70 : .80;
                            const bool final_precision=session_.big() && session_.successfulBigActivations()>=2;
                            const double motion_limit=final_precision?.025:(session_.big()?.040:.018);
                            // This node offers a per-leaf shot intent. Only the
                            // sole output owner checks actual barrel alignment;
                            // independent serial fits can veto each other forever.
                            // Offer one token until the sole command owner
                            // acknowledges it. Timer/callback repeats cannot
                            // create a second bullet, even before a new image.
                            if (!issued_shot_ && motion_.readyToFire() && impact_in_round && target_feedback && pending_current_ && leaves_.available(selected_) &&
                                time >= next_shot_ && motion_.error() < motion_limit) {
                                issued_shot_ = std::max(last_shot_id_ + 1, static_cast<uint64_t>(now().nanoseconds()));
                                last_shot_id_ = issued_shot_;
                                issued_slot_ = selected_;
                                issued_generation_ = leaves_.generation();
                                offers_[issued_shot_] = {selected_, aim.ballistic.flight_time, state_->ammo_remaining,
                                                        leaves_.generation()};
                            }
                            out.shot_id = issued_shot_;
                            auto offered=offers_.find(issued_shot_);
                            if(offered!=offers_.end())offered->second.flight=aim.ballistic.flight_time;
                            out.fire = issued_shot_ && motion_.readyToFire() && (!final_precision || motion_.error()<motion_limit) && impact_in_round && target_feedback && issued_slot_ == selected_ &&
                                       issued_generation_ == leaves_.generation() &&
                                       leaves_.available(selected_) && pending_current_ &&
                                       time + aim.ballistic.flight_time + latency_ - session_.sent() < 20.;
                            out.reason =
                                out.fire
                                    ? "ALLOW_RUNE_SINGLE_SHOT"
                                    : (!barrel.valid
                                           ? "WAIT_GIMBAL_FEEDBACK"
                                           : (time < next_shot_
                                                  ? "WAIT_HIT_IMAGE"
                                                  : (error >= fire_alignment ? "WAIT_ALIGNMENT"
                                                                  : (motion_.error() >= motion_limit ? "WAIT_MOTION_QUALITY"
                                                                                             : "WAIT_PENDING_LEAF"))));
                            if (!pending_current_)
                                out.reason = "HOLD_PENDING_GAP_NO_FIRE";
                            else if (!leaves_.available(selected_))
                                out.reason = session_.big()?"WAIT_BIG_POST_IMPACT_IMAGE":"WAIT_NEW_ILLUMINATION_SINGLE_SHOT";
                            else if (!target_feedback)
                                out.reason = "WAIT_TARGET_FEEDBACK";
                            else if (!impact_in_round)
                                out.reason = session_.big()?leaves_.blockedReason():"WAIT_NEXT_BIG_ROUND_DEADLINE";
                            else if (!motion_.readyToFire())
                                out.reason = "WAIT_BIG_ROTATION_MODEL";
                        }
                    } else if (age > .28 || age < 0) {
                        out.fire = false;
                        out.reason = "STALE_RUNE_IMAGE";
                        // Stationary rune mode: hold the last measured target
                        // direction with fire=false until a new target arrives.
                        // Recentring on R makes reacquisition itself oscillate.
                        out.yaw = last_aim_yaw_;
                        out.pitch = last_aim_pitch_;
                        out.valid = std::isfinite(last_aim_time_);
                    }
                }
            }
        }

        if (freshState()) {
            Status status;
            status.header = out.header;
            status.state_session = out.state_session;
            status.frame_sequence = out.frame_sequence;
            status.goal_id = goal_;
            status.request_id = session_.request();
            status.opportunities = session_.count();
            status.finished = session_.finished();
            status.activated = session_.activated();
            status.reason = session_.reason();
            status.buff_active = buff_active_;
            status.feedback_known = feedback_known_;
            status.request_uncertain = session_.uncertain();

            status_pub_->publish(status);
        }
        // A look-at-hub angle is allowed before acquisition, but never a shot.
        if (!out.fire && out.observation_stamp.sec == 0)
            out.observation_stamp = out.header.stamp;
        if (!ready)
            out.reason = "WAIT_FRESH_BUFF_MISSION";
        last_reason_ = out.reason;

        std::ostringstream detail;
        detail << std::fixed << std::setprecision(2) << "align=" << alignment << " raw=" << raw_alignment
               << " fit=" << motion_.error() << " leaf=" << selected_ << " shot=" << out.shot_id
               << " cross=" << detected_pending_ << " solved=" << solved_leaves_
               << " consistent=" << consistent_leaves_ << " big=" << session_.big()
               << " big_successes=" << session_.successfulBigActivations()
               << " tracking_reject=" << big_feedback_.rejection()
               << " feedback_mode=" << big_feedback_.estimateSource()
               << " feedback_age=" << (state_?now().seconds()-campus::stampSeconds(state_->header.stamp):INFINITY);
        if(session_.big()) detail << " observed_span=" << motion_.bigModel().observedSpan()
            << " fitted_span=" << motion_.bigModel().fittedSpan()
            << " model_ready=" << motion_.bigModel().modelReady()
            << " fit_rms=" << motion_.bigModel().fitError()
            << " validation=" << motion_.bigModel().validationError()
            << " prediction_difference=" << motion_.bigModel().predictionError();
        diagnostic_ = detail.str();

        if (active_)
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "RUNE_GATE %s valid=%d fire=%d pending=%d motion=%d %s", out.reason.c_str(), out.valid,
                                 out.fire, pending_current_, motion_.ready(), diagnostic_.c_str());

        if(active_ && session_.accepted() && out.valid)
            big_feedback_.command(now().seconds()+latency_,out.yaw,out.pitch);
        solution_pub_->publish(out);
    }

    double last_measured_hub_=-INFINITY;
    campus::RuneGimbalFeedback feedback_;
    campus_buff::BigRuneGimbal big_feedback_;

    campus_buff::RuneTargetCycle leaves_;

    struct ShotOffer {
        int slot;
        double flight;
        int ammo;
        uint64_t generation;
    };
    std::map<uint64_t, ShotOffer> offers_;
    uint64_t issued_shot_ = 0, last_shot_id_ = 0;
    uint64_t issued_generation_ = 0;
    uint64_t preferred_generation_ = UINT64_MAX;
    int issued_slot_ = -1, shot_slot_ = -1, shot_ammo_ = 0;
    double shot_stamp_ = 0, shot_time_ = -INFINITY;
    double shot_flight_=0,shot_no_ammo_stamp_=-INFINITY;
    bool shot_confirmed_ = false, extra_shot_noted_ = false;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr shot_sub_;
    int detected_pending_ = 0, solved_leaves_ = 0, consistent_leaves_ = 0;
    std::string last_reason_ = "INACTIVE", diagnostic_;
    BuffDetector detector_;

    campus_buff::RotorMotion motion_;

    campus_buff::RunePlane plane_;

    campus_buff::RuneSession session_;
    campus::FrameGuard guard_;
    cv::Vec3d search_center_{};
    double physical_radius_ = .7;
    bool have_hub_ = false;
    double pending_until_ = 0, last_frame_stamp_ = -1;
    std::string vision_reason_ = "WAIT_RUNE_FRAME";
    bool buff_active_ = false, feedback_known_ = false, cooldown_wait_ = false;
    uint64_t last_accepted_id_ = 0;
    State::ConstSharedPtr state_;
    State observation_;
    bool active_ = false, arrived_ = false, display_ = false, have_observation_ = false, pending_current_ = false;
    uint64_t goal_ = 0, follow_goal_ = 0, last_id_ = 0;
    int selected_ = -1;
    double state_time_ = -INFINITY, mission_time_ = -INFINITY, follow_time_ = -INFINITY, last_retry_ = 0,
           request_stamp_ = 0;
    double last_rotor_stamp_ = -INFINITY, last_pending_stamp_ = -INFINITY, target_change_stamp_ = -INFINITY;
    double last_aim_time_ = -INFINITY, last_aim_yaw_ = 0, last_aim_pitch_ = 0;
    double target_radius_ = .7, nominal_speed_ = 23.5;
    double phase_ = 0, latency_ = .025, next_shot_ = 0;
    double match_start_stamp_=0;
    cv::Vec3d hub_{};
    Request request_;
    rclcpp::Publisher<Solution>::SharedPtr solution_pub_;
    rclcpp::Publisher<Status>::SharedPtr status_pub_;
    rclcpp::Publisher<Request>::SharedPtr request_pub_;
    rclcpp::Subscription<State>::SharedPtr state_sub_;
    rclcpp::Subscription<campus::Frame>::SharedPtr frame_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_, follow_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
    rclcpp::Subscription<tdt_interface::msg::RuneResult>::SharedPtr result_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

namespace {
volatile std::sig_atomic_t stopped = 0;

void signalStop(int) {
    stopped = 1;
}
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, signalStop);
    std::signal(SIGTERM, signalStop);
    auto node = std::make_shared<BuffNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stopped) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (rclcpp::ok())
        node->stop();
    rclcpp::shutdown();
    return 0;
}
