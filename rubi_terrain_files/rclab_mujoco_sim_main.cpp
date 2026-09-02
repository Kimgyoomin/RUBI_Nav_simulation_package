/**
 * @file rclab_mujoco_sim_main.cpp
 * @author YunHo Han (93yunho@gmail.com), Jeong-Hwan Jang(jang990608@gmail.com)
 * @brief Robot Model is RoK-4. MuJoCo 시뮬레이션의 메인 로직을 구현하는 파일입니다.
 * @version 0.5.1
 * @date 2025-10-13
 * @copyright Copyright (c) 2025
 */
#include "rclab_mujoco_sim/rclab_mujoco_sim_main.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kRlTorqueLimit = 90.0f;

Eigen::Vector6f clip_action_to_torque_limit(const RLRobot& robot, const Eigen::VectorXf& action, float torque_limit) 
{
    const float kp = robot.gain.Joint_Kp.mean();
    const float kd = robot.gain.Joint_Kd.mean();
    Eigen::Vector6f clipped_action;

    for (int i = 0; i < 6; ++i)
    {
        const float action_scale = robot.scale.actionScale.diagonal().coeff(i);
        float target_pos = action(i) * action_scale;
        const float lower = robot.joint.ActualPos(i) - robot.joint.DefaultDofPos(i) + (kd * robot.joint.ActualVel(i) - torque_limit) / kp;
        const float upper = robot.joint.ActualPos(i) - robot.joint.DefaultDofPos(i) + (kd * robot.joint.ActualVel(i) + torque_limit) / kp;

        target_pos = std::clamp(target_pos, lower, upper);
        clipped_action(i) = target_pos / action_scale;
    }

    return clipped_action;
}

void feed_velocity_tracking(mjModel* model, mjData* data, rclab_mujoco_ui& ui,
                            float cmd_x, float cmd_y, float cmd_yaw) {
    if (!model || !data) return;

    const int base_body_id = mj_name2id(model, mjOBJ_BODY, "BODY");
    if (base_body_id < 0) return;

    mjtNum base_vel_world[6] = {0, 0, 0, 0, 0, 0};
    mj_objectVelocity(model, data, mjOBJ_BODY, base_body_id, base_vel_world, 0);

    const mjtNum* xmat = data->xmat + 9 * base_body_id;
    const double yaw = std::atan2(static_cast<double>(xmat[3]), static_cast<double>(xmat[0]));
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const double vx_world = static_cast<double>(base_vel_world[3]);
    const double vy_world = static_cast<double>(base_vel_world[4]);

    const float base_x = static_cast<float>(cos_yaw * vx_world + sin_yaw * vy_world);
    const float base_y = static_cast<float>(-sin_yaw * vx_world + cos_yaw * vy_world);
    const float base_yaw = static_cast<float>(base_vel_world[2]);

    ui.add_velocity_data(cmd_x, cmd_y, cmd_yaw, base_x, base_y, base_yaw);
}
}  // namespace

/**
 * @brief 프로그램의 시작점(Entry Point)입니다.
 * @param argc 메인 함수에 전달되는 인자의 수
 * @param argv 메인 함수에 전달되는 인자 배열
 * @return int 프로그램 종료 코드 (0이면 정상 종료)
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mujoco_sim_node");

    // 3. MujocoSim 클래스의 인스턴스를 생성합니다.
    MujocoSim sim(node);

    // 4. 시뮬레이션 초기화 함수를 호출하고, 성공 시 메인 루프를 실행합니다.
    if (sim.init()) 
    {
        sim.run();
    }

    // 5. ROS 시스템을 종료합니다.
    rclcpp::shutdown();
    return 0;
}

/**
 * @brief MujocoSim 클래스의 생성자 구현부입니다.
 */
MujocoSim::MujocoSim(const rclcpp::Node::SharedPtr& node) : node_(node)
{
    // --- MuJoCo 모델 로드 ---
    std::string xml_file_path = "/home/song/.mujoco/mujoco/model/RUBI/rubi.xml";
    model_ = mj_loadXML(xml_file_path.c_str(), nullptr, error_, 1000);
    if (!model_) 
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load MuJoCo model: %s", error_);
        return;
    }
    data_ = mj_makeData(model_);
    RCLCPP_INFO(node_->get_logger(), "MuJoCo model loaded successfully.");

    // --- ROS 2 퍼블리셔, 구독자 및 관절 정보 설정 ---
    joint_state_publisher_ =
        node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    action_publisher_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>("/rl/action", 10);
    joy_subscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1,
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) { ROSJoyMode(msg); });
    cmdvel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });

    // 모델의 모든 조인트를 순회하며 ROS 메시지에 필요한 정보를 추출하고 저장합니다.
    for (int i = 0; i < model_->njnt; ++i) 
    {
        // Hinge(회전) 또는 Slide(직선) 타입의 조인트만 처리합니다.
        if (model_->jnt_type[i] == mjJNT_HINGE || model_->jnt_type[i] == mjJNT_SLIDE) 
        {
            urdf_joint_names_.push_back(mj_id2name(model_, mjOBJ_JOINT, i));
            urdf_joint_qpos_indices_.push_back(model_->jnt_qposadr[i]);
            urdf_joint_qvel_indices_.push_back(model_->jnt_dofadr[i]);
        }
    }
}

/**
 * @brief MujocoSim 클래스의 소멸자 구현부입니다.
 * @note 리소스 해제 순서는 매우 중요합니다. (ImGui -> GLFW -> MuJoCo)
 */
MujocoSim::~MujocoSim() 
{
    if (window_) 
    {
        // ImGui 리소스 해제
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        // GLFW 리소스 해제
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
    // MuJoCo 리소스 해제
    mjr_freeContext(&rclab_ui_.con);
    mjv_freeScene(&rclab_ui_.scn);
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    RCLCPP_INFO(node_->get_logger(), "Cleaned up all resources.");
}

/**
 * @brief 시뮬레이션 메인 루프입니다.
 */
void MujocoSim::run() 
{
    double sim_timestep = model_->opt.timestep;
    double last_update_time = glfwGetTime();
    double accumulator = 0.0;

    while (rclcpp::ok() && !glfwWindowShouldClose(window_)) 
    {
        double current_time = glfwGetTime();
        double frame_time = current_time - last_update_time;
        last_update_time = current_time;
        accumulator += frame_time;

        if (rclab_ui_.full_reset_request) {
            reset_to_startup_state();
            rclab_ui_.full_reset_request = false;
            accumulator = 0.0;
            last_update_time = glfwGetTime();
        }

        if (rclab_ui_.is_paused && !was_paused_last_frame_) {
            if (state_history_.empty()) {
                capture_current_state();
            }
            if (!state_history_.empty()) {
                pause_anchor_snapshot_ = state_history_[history_cursor_];
                pause_anchor_cursor_ = history_cursor_;
                pause_anchor_valid_ = true;
            }
        } else if (!rclab_ui_.is_paused && was_paused_last_frame_) {
            if (pause_anchor_valid_) {
                restore_snapshot(pause_anchor_snapshot_);
                clear_state_history();
                capture_current_state();
            }
            pause_anchor_valid_ = false;
        }
        was_paused_last_frame_ = rclab_ui_.is_paused;

        if (rclab_ui_.step_backward_request) {
            rclab_ui_.is_paused = true;
            step_paused_history(rclab_ui_.manual_step_duration, -1);
            rclab_ui_.step_backward_request = false;
            accumulator = 0.0;
            last_update_time = glfwGetTime();
        }

        if (rclab_ui_.step_forward_request) {
            rclab_ui_.is_paused = true;
            step_paused_history(rclab_ui_.manual_step_duration, 1);
            rclab_ui_.step_forward_request = false;
            accumulator = 0.0;
            last_update_time = glfwGetTime();
        }

        // 한 스텝씩 실행 요청 처리
        if (rclab_ui_.single_step_request) 
        {
            if (!rclab_ui_.is_paused) 
            {
                rclab_ui_.is_paused = true; // 연속 실행 중이었다면 일시정지
            }
            step_paused_history(sim_timestep, 1);
            if (!state_history_.empty()) {
                pause_anchor_snapshot_ = state_history_[history_cursor_];
                pause_anchor_cursor_ = history_cursor_;
                pause_anchor_valid_ = true;
            }
            rclab_ui_.single_step_request = false;
            accumulator = 0.0; // 누적된 시간 초기화
        }

        // 누적된 실제 시간만큼 물리 스텝을 진행
        while (accumulator >= sim_timestep) 
        {
            if (!rclab_ui_.is_paused) 
            {
                trim_future_state_history();
                advance_simulation();
                capture_current_state();
            }
            accumulator -= sim_timestep;
        }
        
        publish_joint_states();
        rclab_ui_.policy_on_toggle = (RLR.cmdFlag == POLICY_ON);
        rclab_ui_.update_UI_and_render(window_);

        if (rclab_ui_.policy_on_request) 
        {
            RLR.cmdFlag = POLICY_ON;
            emergency_stop = false;
            RCLCPP_INFO(node_->get_logger(), "[UI] POLICY_ON triggered by button");
            rclab_ui_.policy_on_request = false;
        }

        rclcpp::spin_some(node_);
    }
}

void MujocoSim::reset_to_startup_state()
{
    if (!model_ || !data_) return;

    mj_resetData(model_, data_);
    rclab_ui_.reset_to_initial_camera();
    rclab_ui_.pert.active = 0;
    rclab_ui_.pert.select = -1;
    rclab_ui_.is_perturbing = false;
    rclab_ui_.is_paused = false;
    rclab_ui_.single_step_request = false;
    rclab_ui_.step_backward_request = false;
    rclab_ui_.step_forward_request = false;
    rclab_ui_.show_help_overlay = false;
    rclab_ui_.show_info_overlay = true;
    rclab_ui_.follow_robot = false;
    rclab_ui_.policy_on_toggle = false;
    rclab_ui_.policy_on_request = false;
    rclab_ui_.reset_manual_command();
    rclab_ui_.clear_velocity_data();
    pause_anchor_valid_ = false;
    pause_anchor_cursor_ = 0;
    was_paused_last_frame_ = false;

    emergency_stop = false;
    decimation = 0;
    is_walking_mode = false;
    policy_start_time = 0.0;
    current_gait_index_ = 0.0;
    tracked_cmd_vel_x_ = 0.0f;
    tracked_cmd_vel_y_ = 0.0f;
    tracked_cmd_yaw_ = 0.0f;

    RLR.InitializeJoint();
    RLR.Set_Joint_PD_gain();
    RLR.walkready_elapsed_ = 0.0f;
    INIT_Network3();
    RLR.flag.walkready_fin = false;
    RLR.cmdFlag = WALK_READY;

    rclab_ui_.desired_qpos = rclab_ui_.initial_desired_qpos;
    rclab_ui_.kp_gains = rclab_ui_.initial_kp_gains;
    rclab_ui_.kd_gains = rclab_ui_.initial_kd_gains;
    for (size_t i = 0; i < rclab_ui_.apply_ui_desired_qpos.size(); ++i) {
        rclab_ui_.apply_ui_desired_qpos[i] = false;
        rclab_ui_.apply_ui_gains[i] = false;
    }

    mj_forward(model_, data_);
    clear_state_history();
    capture_current_state();
}

void MujocoSim::configure_state_history()
{
    if (!model_) return;

    integration_state_size_ = mj_stateSize(model_, mjSTATE_INTEGRATION);
    const double history_window_seconds = 5.0;
    max_history_size_ = std::max<size_t>(
        2, static_cast<size_t>(std::ceil(history_window_seconds / model_->opt.timestep)) + 1);
}

void MujocoSim::clear_state_history()
{
    state_history_.clear();
    history_cursor_ = 0;
}

void MujocoSim::trim_future_state_history()
{
    if (state_history_.empty()) return;
    if (history_cursor_ + 1 < state_history_.size()) {
        state_history_.erase(state_history_.begin() + static_cast<std::ptrdiff_t>(history_cursor_ + 1),
                             state_history_.end());
    }
}

void MujocoSim::capture_current_state()
{
    if (!model_ || !data_) return;
    if (integration_state_size_ <= 0 || max_history_size_ == 0) {
        configure_state_history();
    }

    trim_future_state_history();

    SimulationSnapshot snapshot;
    snapshot.mj_state.resize(integration_state_size_);
    mj_getState(model_, data_, snapshot.mj_state.data(), mjSTATE_INTEGRATION);
    snapshot.velocity_history = rclab_ui_.capture_velocity_history();
    snapshot.rlr_state = RLR;
    snapshot.obs_history_queue = obsHistoryQueue_;
    snapshot.current_obs = currentObs_;
    snapshot.current_actor_obs = currentActorObs_;
    snapshot.encoder_output = encoderOutput_;
    snapshot.policy_input = policyInput_;
    snapshot.obs_history_flat = obsHistoryFlat_;
    snapshot.decimation = decimation;
    snapshot.emergency_stop = emergency_stop;
    snapshot.is_walking_mode = is_walking_mode;
    snapshot.policy_start_time = policy_start_time;
    snapshot.current_gait_index = current_gait_index_;
    snapshot.tracked_cmd_vel_x = tracked_cmd_vel_x_;
    snapshot.tracked_cmd_vel_y = tracked_cmd_vel_y_;
    snapshot.tracked_cmd_yaw = tracked_cmd_yaw_;

    state_history_.push_back(std::move(snapshot));
    if (state_history_.size() > max_history_size_) {
        state_history_.pop_front();
    }
    history_cursor_ = state_history_.empty() ? 0 : state_history_.size() - 1;
}

void MujocoSim::restore_snapshot(const SimulationSnapshot& snapshot)
{
    mj_setState(model_, data_, snapshot.mj_state.data(), mjSTATE_INTEGRATION);
    mj_forward(model_, data_);

    RLR = snapshot.rlr_state;
    obsHistoryQueue_ = snapshot.obs_history_queue;
    currentObs_ = snapshot.current_obs;
    currentActorObs_ = snapshot.current_actor_obs;
    encoderOutput_ = snapshot.encoder_output;
    policyInput_ = snapshot.policy_input;
    obsHistoryFlat_ = snapshot.obs_history_flat;
    decimation = snapshot.decimation;
    emergency_stop = snapshot.emergency_stop;
    is_walking_mode = snapshot.is_walking_mode;
    policy_start_time = snapshot.policy_start_time;
    current_gait_index_ = snapshot.current_gait_index;
    tracked_cmd_vel_x_ = snapshot.tracked_cmd_vel_x;
    tracked_cmd_vel_y_ = snapshot.tracked_cmd_vel_y;
    tracked_cmd_yaw_ = snapshot.tracked_cmd_yaw;
    rclab_ui_.restore_velocity_history(snapshot.velocity_history);
    rclab_ui_.policy_on_toggle = (RLR.cmdFlag == POLICY_ON);
}

void MujocoSim::restore_state_from_history_index(size_t index)
{
    if (!model_ || !data_ || index >= state_history_.size()) return;

    const SimulationSnapshot& snapshot = state_history_[index];
    restore_snapshot(snapshot);
    history_cursor_ = index;
}

int MujocoSim::step_count_from_duration(double duration_seconds) const
{
    if (!model_ || model_->opt.timestep <= 0.0) return 1;
    return std::max(1, static_cast<int>(std::llround(duration_seconds / model_->opt.timestep)));
}

void MujocoSim::step_paused_history(double duration_seconds, int direction)
{
    if (!model_ || !data_) return;
    if (state_history_.empty()) {
        capture_current_state();
    }

    const int requested_steps = step_count_from_duration(duration_seconds);
    if (direction < 0) {
        const size_t step_count = static_cast<size_t>(requested_steps);
        const size_t target_index = (history_cursor_ > step_count) ? (history_cursor_ - step_count) : 0;
        restore_state_from_history_index(target_index);
        return;
    }

    size_t anchor_cursor = history_cursor_;
    if (pause_anchor_valid_) {
        anchor_cursor = std::min(pause_anchor_cursor_, state_history_.size() - 1);
    }

    const size_t current_cursor = history_cursor_;
    size_t preview_steps = 0;
    if (current_cursor < anchor_cursor) {
        preview_steps = std::min(static_cast<size_t>(requested_steps), anchor_cursor - current_cursor);
        restore_state_from_history_index(current_cursor + preview_steps);
    }

    const int remaining_steps = requested_steps - static_cast<int>(preview_steps);
    if (remaining_steps <= 0) return;

    if (pause_anchor_valid_) {
        restore_snapshot(pause_anchor_snapshot_);
        history_cursor_ = std::min(anchor_cursor, state_history_.size() - 1);
    }

    trim_future_state_history();
    for (int i = 0; i < remaining_steps; ++i) {
        advance_simulation();
        capture_current_state();
        if (pause_anchor_valid_ && !state_history_.empty()) {
            pause_anchor_snapshot_ = state_history_[history_cursor_];
            pause_anchor_cursor_ = history_cursor_;
        }
    }
}

/**
 * @brief [수정] RL 제어 로직을 포함하는 시뮬레이션 스텝 함수입니다.
 */
void MujocoSim::advance_simulation() 
{ 
    switch (RLR.cmdFlag) 
    {        
        case WALK_READY:
            RLR.walkready();
            break;

        case POLICY_ON:
            // RLR.Set_Joint_PD_gain();
            if(RLR.flag.walkready_fin == true)
            {
                if (decimation % 5 == 0) 
                { 
                    // Inference2(); standup
                    Inference3();
                }
            }
            break;

        case TORQUE_OFF:
            emergency_stop = true;
            break;
    }

    EncoderRead();
    
    JointPDTorqueControl_RL();
    
    if (decimation % 5 == 0) 
    { 
        IMUSensorRead();
    }

    decimation++;

    rclab_ui_.resolve_command(RLR.obs.joy_cmd_x, RLR.obs.joy_cmd_y, RLR.obs.joy_cmd_z,
                              tracked_cmd_vel_x_, tracked_cmd_vel_y_, tracked_cmd_yaw_);
    mjv_applyPerturbForce(model_, data_, &rclab_ui_.pert); // UI를 통한 외란 적용
    mj_step(model_, data_);  // MuJoCo 물리 엔진 한 스텝 진행

    feed_velocity_tracking(model_, data_, rclab_ui_,
                           tracked_cmd_vel_x_,
                           tracked_cmd_vel_y_,
                           tracked_cmd_yaw_);
}

/**
 * @brief 시뮬레이션 상세 초기화 함수입니다.
 */
bool MujocoSim::init() 
{
    if (!model_) return false;  // 생성자에서 모델 로딩 실패 시 초기화 중단

    // --- 1. GLFW 초기화 및 윈도우 생성 ---
    if (!glfwInit()) 
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not initialize GLFW.");
        return false;
    }
    window_ = glfwCreateWindow(1920, 1080, "RUBI MuJoCo Simulation", NULL, NULL);
    if (!window_) 
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create GLFW window.");
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // V-Sync 활성화

    // --- 2. ImGui 초기화 ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui_ImplGlfw_InitForOpenGL(window_, false);  // 콜백은 수동으로 설치할 것이므로 false
    ImGui_ImplOpenGL3_Init("#version 130");

    // --- 3. ImGui 스타일 및 폰트 설정 ---
    const char* font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
    io.Fonts->AddFontFromFileTTF(font_path, 16.0f);
    const char* bold_font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf";
    rclab_ui_.bold_font = io.Fonts->AddFontFromFileTTF(bold_font_path, 16.0f);
    if (!rclab_ui_.bold_font) 
    {
        RCLCPP_WARN(node_->get_logger(), "Could not load bold font. Please check the file path.");
    }
    rclab_ui_.setup_custom_imgui_style();

    // --- 4. MuJoCo 렌더링 관련 초기화 ---
    rclab_ui_.reset_to_initial_camera();
    mjv_defaultOption(&rclab_ui_.opt);
    for (int i = 0; i < mjNGROUP; i++) rclab_ui_.opt.geomgroup[i] = 1;  // 모든 geom 그룹을 보이게 설정
    rclab_ui_.opt.flags[mjVIS_SELECT] = 1;                              // 마우스 선택 시각화 활성화
    mjv_defaultScene(&rclab_ui_.scn);
    mjr_defaultContext(&rclab_ui_.con);
    mjv_makeScene(model_, &rclab_ui_.scn, 2000);               // 씬 생성 (최대 2000개의 오브젝트)
    mjr_makeContext(model_, &rclab_ui_.con, mjFONTSCALE_100);  // 렌더링 컨텍스트 생성
    mjv_defaultPerturb(&rclab_ui_.pert);

    // --- 5. GLFW 콜백 함수 설정 ---
    user_pointers_ = {model_, data_, &rclab_ui_};
    glfwSetWindowUserPointer(window_, &user_pointers_);
    glfwSetKeyCallback(window_, rclab_mujoco_ui::keyboard);
    glfwSetCursorPosCallback(window_, rclab_mujoco_ui::mouse_move);
    glfwSetMouseButtonCallback(window_, rclab_mujoco_ui::mouse_button);
    glfwSetScrollCallback(window_, rclab_mujoco_ui::scroll);
    glfwSetCharCallback(window_, rclab_mujoco_ui::char_callback);

    // --- 6. RL 제어기 변수 초기화 --- // by gemini
    RLR.InitializeJoint();
    RLR.Set_Joint_PD_gain();
    // RLR.walk_PD_gain();
    // INIT_Network();
    INIT_Network3();
    RLR.flag.walkready_fin = false;
    RLR.cmdFlag = WALK_READY;

    // --- 7. UI 및 기타 변수 초기화 --- // by gemini
    int num_actuators = model_->nu;
    // PD Control UI를 위한 변수 초기화 (충돌 방지)
    rclab_ui_.desired_qpos.resize(num_actuators, 0.0);
    rclab_ui_.initial_desired_qpos.resize(num_actuators, 0.0);
    rclab_ui_.kp_gains.resize(num_actuators, 0.0); // 초기 게인값은 RL 컨트롤러가 덮어쓰므로 0으로 설정 가능
    rclab_ui_.kd_gains.resize(num_actuators, 0.0);
    rclab_ui_.initial_kp_gains.resize(num_actuators, 0.0);
    rclab_ui_.initial_kd_gains.resize(num_actuators, 0.0);
    rclab_ui_.apply_ui_desired_qpos.resize(num_actuators, false);
    rclab_ui_.apply_ui_gains.resize(num_actuators, false);

    // 액추에이터 주소 매핑
    actuator_qpos_adr_.resize(num_actuators, -1);
    actuator_qvel_adr_.resize(num_actuators, -1);
    for (int i = 0; i < num_actuators; ++i) 
    {
        if (model_->actuator_trntype[i] == mjTRN_JOINT) 
        {
            int trn_id = model_->actuator_trnid[i * 2];
            actuator_qpos_adr_[i] = model_->jnt_qposadr[trn_id];
            actuator_qvel_adr_[i] = model_->jnt_dofadr[trn_id];
            joint_dof_to_actuator_map_[model_->jnt_dofadr[trn_id]] = i;
            // UI에 표시될 초기 목표 위치 설정
            rclab_ui_.desired_qpos[i] = model_->qpos0[actuator_qpos_adr_[i]];
            rclab_ui_.kp_gains[i] = static_cast<double>(RLR.gain.Joint_Kp[i]);
            rclab_ui_.kd_gains[i] = static_cast<double>(RLR.gain.Joint_Kd[i]);
        }
    }
    rclab_ui_.initial_desired_qpos = rclab_ui_.desired_qpos;
    rclab_ui_.initial_kp_gains = rclab_ui_.kp_gains;
    rclab_ui_.initial_kd_gains = rclab_ui_.kd_gains;

    configure_state_history();
    clear_state_history();
    mj_forward(model_, data_);
    capture_current_state();

    return true;  // 모든 초기화 성공
}

// --- RL Control Functions (from rubi_plugin.cc) --- //
void MujocoSim::INIT_Network3()
{
    static bool models_loaded = false;
    // ==== SET ONNX ==== //
    RLR.obs.raw_joy_cmd_x = 0.0f;
    RLR.obs.raw_joy_cmd_y = 0.0f;
    RLR.obs.raw_joy_cmd_z = 0.0f;

    RLR.obs.joy_cmd_x = 0.0f;
    RLR.obs.joy_cmd_y = 0.0f;
    RLR.obs.joy_cmd_z = 0.0f;

    if (!models_loaded) 
    {
        const std::string encoderModelPath =
            std::string(RCLAB_MUJOCO_SIM_PACKAGE_DIR) + "/src/onnx_data/encoder.onnx";
        const std::string policyModelPath =
            std::string(RCLAB_MUJOCO_SIM_PACKAGE_DIR) + "/src/onnx_data/policy.onnx";

        RCLCPP_INFO(node_->get_logger(), "Loading encoder model...");
        onnx_encoder.ONNX_LOAD(encoderModelPath.c_str());

        RCLCPP_INFO(node_->get_logger(), "Loading policy model...");
        onnx_policy.ONNX_LOAD(policyModelPath.c_str());
        RCLCPP_INFO(node_->get_logger(), "Successfully loaded 2 ONNX models!");
        models_loaded = true;
    }

    // ===== zero-input ONNX check =====
    // Eigen::VectorXf zeroObsHistory = Eigen::VectorXf::Zero(encoderInputSize_); // 330
    // Eigen::VectorXf zeroActorObs = Eigen::VectorXf::Zero(actorObsSize_);       // 33

    // Eigen::VectorXf zeroLatent = onnx_encoder.ONNX_INFERENCE(zeroObsHistory); // 32

    // Eigen::VectorXf zeroPolicyInput(policyInputSize_);                        // 65
    // zeroPolicyInput << zeroLatent, zeroActorObs;

    // Eigen::VectorXf zeroActions = onnx_policy.ONNX_INFERENCE(zeroPolicyInput); // 6

    // std::cout << "zeroLatent: " << zeroLatent.transpose() << std::endl;
    // std::cout << "zeroPolicyInput: " << zeroPolicyInput.transpose() << std::endl;
    // std::cout << "zeroActions: " << zeroActions.transpose() << std::endl;

    // std::exit(0);

    RLR.time.control_dt                 = 0.01;
    RLR.time.cycle_time                 = 0.40;

    RLR.scale.dofPositionScale          = 1.0;
    RLR.scale.dofVelocityScale          = 0.1;
    RLR.scale.angularVelocityScale      = 0.25;
    RLR.scale.linearVelocityScale       = 2.0;
    
    RLR.obs.cmd_x                       = 0.;
    RLR.obs.cmd_y                       = 0.;
    RLR.obs.cmd_z                       = 0.;
    RLR.obs.action                      << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    RLR.obs.pre_action                  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    RLR.obs.filtered_action             << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    RLR.scale.actionScale.diagonal()    << 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f;

    // ==== 버퍼 초기화 ==== //
    currentObs_.resize(observationSize_);
    currentActorObs_.resize(actorObsSize_);
    encoderOutput_.resize(encoderOutputSize_);
    policyInput_.resize(policyInputSize_);
    obsHistoryFlat_.resize(encoderInputSize_);
    currentObs_.setZero();
    currentActorObs_.setZero();
    obsHistoryQueue_.clear();

    for (int i = 0; i < obsHistoryLength_; ++i) 
    {
        obsHistoryQueue_.push_back(Eigen::VectorXf::Zero(actorObsSize_));
    }

}

void MujocoSim::Inference3()
{
    // --- 1. 현재 관측(Observation) 계산 ---
    
    static Eigen::Vector3f projected_gravity;
    static Eigen::Vector3f imu_scaled_angvel;

    projected_gravity                   = RLR.obs.Projected_Gravity;
    imu_scaled_angvel                   = RLR.scale.angularVelocityScale  * RLR.imu.AngularVel;
    RLR.joint.Scaled_ActualPos          = RLR.scale.dofPositionScale      * (RLR.joint.ActualPos - RLR.joint.DefaultDofPos);
    RLR.joint.Scaled_ActualVel          = RLR.scale.dofVelocityScale      * RLR.joint.ActualVel;
    RLR.joint.Actions                   = RLR.obs.pre_action; 

    rclab_ui_.resolve_command(RLR.obs.joy_cmd_x, RLR.obs.joy_cmd_y, RLR.obs.joy_cmd_z,
                              RLR.obs.cmd_x, RLR.obs.cmd_y, RLR.obs.cmd_z);

    RLR.obs.commands    << RLR.obs.cmd_x, RLR.obs.cmd_y, RLR.obs.cmd_z;

    Eigen::Vector3f scaled_commands;
    scaled_commands << RLR.obs.cmd_x * RLR.scale.linearVelocityScale,  // 1.0 * 2.0 = 2.0
                       RLR.obs.cmd_y * RLR.scale.linearVelocityScale,  // 0.75 * 2.0 = 1.5
                       RLR.obs.cmd_z * RLR.scale.angularVelocityScale; // 1.5 * 0.25 = 0.375


    // frequencies = [1.5, 2.5] -> 중간값 2.0
    const double GAIT_FREQUENCY = 2.0; 
    
    // offsets = 0.5 
    const double GAIT_OFFSET = 0.5; 
    
    // durations = [0.5, 0.5] -> 고정값 0.5
    const double GAIT_DURATION = 0.5; 
    
    // swing_height = [0.0, 0.1] -> 중간값 0.05 (혹은 0.1 등 원하는 값)
    const double GAIT_SWING_HEIGHT = 0.0;

    // rubi.py의 gait 로직 (시간 기반)
    // rubi.py의 obs: ang_vel(3), gravity(3), dof_pos(6), dof_vel(6), actions(6), clock_sin(1), clock_cos(1), gaits(4) = 30
    
    current_gait_index_ += RLR.time.control_dt * GAIT_FREQUENCY;
    if (current_gait_index_ > 1.0) 
    {
        current_gait_index_ -= 1.0;
    }
    float clock_sin = sin(current_gait_index_ * 2 * M_PI);
    float clock_cos = cos(current_gait_index_ * 2 * M_PI);
    
    Eigen::VectorXf gait_params(4);
    gait_params << GAIT_FREQUENCY, 
                   GAIT_OFFSET, 
                   GAIT_DURATION, 
                   GAIT_SWING_HEIGHT;

    currentObs_.resize(observationSize_); // 30
    currentObs_ <<  imu_scaled_angvel,
                    projected_gravity,
                    RLR.joint.Scaled_ActualPos,  
                    RLR.joint.Scaled_ActualVel,
                    RLR.joint.Actions,
                    clock_sin,
                    clock_cos,
                    gait_params;

    // CTS actor observation = proprioceptive observation + scaled command.
    currentActorObs_.resize(actorObsSize_);
    currentActorObs_ << currentObs_, scaled_commands;

    // --- 2. Actor Observation History 업데이트 ---
    obsHistoryQueue_.pop_front(); // 가장 오래된 obs 제거
    obsHistoryQueue_.push_back(currentActorObs_); // 가장 최신 actor obs 추가

    // 1D 벡터(obsHistoryFlat_)로 펼치기
    for (int i = 0; i < obsHistoryLength_; ++i) 
    {
        obsHistoryFlat_.segment(i * actorObsSize_, actorObsSize_) = obsHistoryQueue_[i];
    }

    // --- 3. Encoder 추론 ---
    // (onnx_encoder 객체 사용)
    encoderOutput_ = onnx_encoder.ONNX_INFERENCE(obsHistoryFlat_);

    // Policy 입력 벡터 생성 (latent + actor obs)
    policyInput_.resize(policyInputSize_); // 65
    policyInput_ << encoderOutput_,   // 32
                    currentActorObs_; // 33

    // (onnx_policy 객체 사용)
    Eigen::VectorXf policyOutput = onnx_policy.ONNX_INFERENCE(policyInput_);

    RLR.obs.action = policyOutput;
    
    // --- 5. Action 필터링 (기존 로직 유지) ---
    float alpha = 1.0;    
    for (int i=0; i < 6; i++)
    {
        RLR.obs.filtered_action(i) = (1-alpha) * RLR.obs.pre_action(i) + alpha * RLR.obs.action(i);
    }

    RLR.obs.filtered_action = clip_action_to_torque_limit(RLR, RLR.obs.filtered_action, kRlTorqueLimit);
    
    RLR.obs.pre_action = RLR.obs.filtered_action;
    
    
}

void MujocoSim::JointPDTorqueControl_RL()
{   
    if(emergency_stop == true)
    {
        for(size_t i = 0; i < 6; ++i)
        {
            RLR.joint.RefTorque[i] = 0.0;
        }
    }
    else
    {
        if(RLR.flag.walkready_fin == false)
        {
            RLR.joint.TargetPos[RLR.J_LHR] = RLR.joint.walkreadyTargetPos[RLR.J_LHR];
            RLR.joint.TargetPos[RLR.J_LHP] = RLR.joint.walkreadyTargetPos[RLR.J_LHP];
            RLR.joint.TargetPos[RLR.J_LKN] = RLR.joint.walkreadyTargetPos[RLR.J_LKN];

            RLR.joint.TargetPos[RLR.J_RHR] = RLR.joint.walkreadyTargetPos[RLR.J_RHR];
            RLR.joint.TargetPos[RLR.J_RHP] = RLR.joint.walkreadyTargetPos[RLR.J_RHP];
            RLR.joint.TargetPos[RLR.J_RKN] = RLR.joint.walkreadyTargetPos[RLR.J_RKN];
        }
        else
        {
            if(RLR.cmdFlag == POLICY_ON)
            {   
                RLR.joint.TargetPos[RLR.J_LHR] = RLR.scale.actionScale.diagonal().coeff(RLR.J_LHR) * RLR.obs.filtered_action[RLR.J_LHR] + RLR.joint.DefaultDofPos[RLR.J_LHR];
                RLR.joint.TargetPos[RLR.J_LHP] = RLR.scale.actionScale.diagonal().coeff(RLR.J_LHP) * RLR.obs.filtered_action[RLR.J_LHP] + RLR.joint.DefaultDofPos[RLR.J_LHP];
                RLR.joint.TargetPos[RLR.J_LKN] = RLR.scale.actionScale.diagonal().coeff(RLR.J_LKN) * RLR.obs.filtered_action[RLR.J_LKN] + RLR.joint.DefaultDofPos[RLR.J_LKN];

                RLR.joint.TargetPos[RLR.J_RHR] = RLR.scale.actionScale.diagonal().coeff(RLR.J_RHR) * RLR.obs.filtered_action[RLR.J_RHR] + RLR.joint.DefaultDofPos[RLR.J_RHR];
                RLR.joint.TargetPos[RLR.J_RHP] = RLR.scale.actionScale.diagonal().coeff(RLR.J_RHP) * RLR.obs.filtered_action[RLR.J_RHP] + RLR.joint.DefaultDofPos[RLR.J_RHP];
                RLR.joint.TargetPos[RLR.J_RKN] = RLR.scale.actionScale.diagonal().coeff(RLR.J_RKN) * RLR.obs.filtered_action[RLR.J_RKN] + RLR.joint.DefaultDofPos[RLR.J_RKN];
            }
            else
            {
                RLR.joint.TargetPos[RLR.J_LHR] = RLR.joint.walkreadyTargetPos[RLR.J_LHR];
                RLR.joint.TargetPos[RLR.J_LHP] = RLR.joint.walkreadyTargetPos[RLR.J_LHP];
                RLR.joint.TargetPos[RLR.J_LKN] = RLR.joint.walkreadyTargetPos[RLR.J_LKN];

                RLR.joint.TargetPos[RLR.J_RHR] = RLR.joint.walkreadyTargetPos[RLR.J_RHR];
                RLR.joint.TargetPos[RLR.J_RHP] = RLR.joint.walkreadyTargetPos[RLR.J_RHP];
                RLR.joint.TargetPos[RLR.J_RKN] = RLR.joint.walkreadyTargetPos[RLR.J_RKN];
            }
        }
        for (int i = 0; i < model_->nu && i < static_cast<int>(rclab_ui_.desired_qpos.size()); ++i) {
            if (i < static_cast<int>(rclab_ui_.apply_ui_desired_qpos.size()) && rclab_ui_.apply_ui_desired_qpos[i]) {
                RLR.joint.TargetPos[i] = static_cast<float>(rclab_ui_.desired_qpos[i]);
            }
            if (i < static_cast<int>(rclab_ui_.apply_ui_gains.size()) && rclab_ui_.apply_ui_gains[i]) {
                RLR.gain.Joint_Kp[i] = static_cast<float>(rclab_ui_.kp_gains[i]);
                RLR.gain.Joint_Kd[i] = static_cast<float>(rclab_ui_.kd_gains[i]);
            }
        }
        for(size_t i = 0; i < 6; ++i)
        {
            RLR.joint.RefTorque[i] = RLR.gain.Joint_Kp[i]*(RLR.joint.TargetPos[i] - RLR.joint.ActualPos[i]) + RLR.gain.Joint_Kd[i]*(0.0 - RLR.joint.ActualVel[i]);
            RLR.joint.RefTorque[i] = std::clamp(RLR.joint.RefTorque[i], -kRlTorqueLimit, kRlTorqueLimit);
            data_->ctrl[i] = RLR.joint.RefTorque[i]; // Apply torque to simulation
        }
    }
}

void MujocoSim::ROSJoyMode(const sensor_msgs::msg::Joy::SharedPtr &msg)
{
    if (msg->axes.size() < 4 || msg->buttons.size() < 10) 
    { // by gemini
        RCLCPP_WARN_ONCE(node_->get_logger(), "Joystick message has an unexpected number of axes or buttons. Skipping.");
        return;
    }

    static bool JoyMode = 0.0;
    if(msg->buttons.size() == 13){JoyMode = 0; /*SERIAL*/}
    else{JoyMode = 1; /*BLUETOOTH*/}
    
    if(JoyMode == 0 /*SERIAL*/)
    {
        if(msg->buttons[8] == true)
        {
            RLR.cmdFlag = POLICY_ON;
        }
        else if(msg->buttons[9] == true)
        {
            RLR.cmdFlag = TORQUE_OFF;
        }
    }

    RLR.obs.raw_joy_cmd_x = 1.0 * (msg->axes[1]);
    RLR.obs.raw_joy_cmd_y = 0.75 * (msg->axes[0]);
    RLR.obs.raw_joy_cmd_z = 1.5 * (msg->axes[3]);

    RLR.obs.joy_cmd_x = RLR.obs.raw_joy_cmd_x;
    RLR.obs.joy_cmd_y = RLR.obs.raw_joy_cmd_y;
    RLR.obs.joy_cmd_z = RLR.obs.raw_joy_cmd_z;

    tracked_cmd_vel_x_ = RLR.obs.raw_joy_cmd_x;
    tracked_cmd_vel_y_ = RLR.obs.raw_joy_cmd_y;
    tracked_cmd_yaw_ = RLR.obs.raw_joy_cmd_z;
}

void MujocoSim::IMUSensorRead() 
{
    // --- Gyroscope ---
    int gyro_id = mj_name2id(model_, mjOBJ_SENSOR, "angular_velocity"); // Use user-provided name
    if (gyro_id != -1) 
    {
        int gyro_adr = model_->sensor_adr[gyro_id];
        RLR.imu.AngularVel[0] = data_->sensordata[gyro_adr];
        RLR.imu.AngularVel[1] = data_->sensordata[gyro_adr+1];
        RLR.imu.AngularVel[2] = data_->sensordata[gyro_adr+2];
    } else 
    {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Gyroscope sensor 'angular_velocity' not found in the model.");
        RLR.imu.AngularVel.setZero();
    }

    // --- Accelerometer & Quaternion (for Projected Gravity) ---
    Eigen::Vector4d orientation_q_IMU_data;
    orientation_q_IMU_data.setZero();
    orientation_q_IMU_data[3] = 1.0; // Default to identity quaternion (w=1)

    int quat_id = mj_name2id(model_, mjOBJ_SENSOR, "imu_quat");
    if (quat_id != -1) 
    {
        int quat_adr = model_->sensor_adr[quat_id];
        orientation_q_IMU_data[3] = data_->sensordata[quat_adr];   // w
        orientation_q_IMU_data[0] = data_->sensordata[quat_adr+1]; // x
        orientation_q_IMU_data[1] = data_->sensordata[quat_adr+2]; // y
        orientation_q_IMU_data[2] = data_->sensordata[quat_adr+3]; // z
    } else 
    {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Quaternion sensor 'imu_quat' not found. Using identity quaternion.");
    }

    Eigen::Vector3d unit_vector(0, 0, -1);
    RLR.imu.convert_data = RLR.quat_rotate_inverse(orientation_q_IMU_data, unit_vector);
    RLR.obs.Projected_Gravity = RLR.imu.convert_data.cast<float>();

    int accel_id = mj_name2id(model_, mjOBJ_SENSOR, "imu_accel");
    if (accel_id != -1) 
    {
        int accel_adr = model_->sensor_adr[accel_id];
        RLR.imu.LinearAcc[0] = data_->sensordata[accel_adr];
        RLR.imu.LinearAcc[1] = data_->sensordata[accel_adr+1];
        RLR.imu.LinearAcc[2] = data_->sensordata[accel_adr+2];
    } else 
    {
        RCLCPP_WARN_ONCE(node_->get_logger(), "Accelerometer sensor 'imu_accel' not found in the model.");
        RLR.imu.LinearAcc.setZero();
    }
}

void MujocoSim::EncoderRead() 
{
    for (int i = 0; i < model_->nu; ++i) 
    {
        if (actuator_qpos_adr_[i] != -1) 
        {
            RLR.joint.ActualPos[i] = data_->qpos[actuator_qpos_adr_[i]];
        }
        if (actuator_qvel_adr_[i] != -1) 
        {
            RLR.joint.ActualVel[i] = data_->qvel[actuator_qvel_adr_[i]];
        }
        RLR.joint.ActualTorque[i] = data_->actuator_force[i];
    }
}

/**
 * @brief 현재 관절 상태를 ROS 1 토픽으로 발행합니다.
 */
void MujocoSim::publish_joint_states() 
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = urdf_joint_names_;

    // 미리 저장된 인덱스를 사용하여 qpos와 qvel에서 값을 읽어 메시지를 채웁니다.
    for (const auto& i : urdf_joint_qpos_indices_) msg.position.push_back(data_->qpos[i]);
    for (const auto& i : urdf_joint_qvel_indices_) msg.velocity.push_back(data_->qvel[i]);

    // 실제 적용된 토크(effort)를 채웁니다.
    msg.effort.reserve(urdf_joint_names_.size());
    for (const auto& dof_adr : urdf_joint_qvel_indices_) 
    {
        auto it = joint_dof_to_actuator_map_.find(dof_adr);
        if (it != joint_dof_to_actuator_map_.end()) 
        {
            int actuator_id = it->second;
            msg.effort.push_back(data_->actuator_force[actuator_id]);
        } else 
        {
            msg.effort.push_back(0.0);
        }
    }

    joint_state_publisher_->publish(msg);

    // =======================
    // [추가] RL action 퍼블리시
    // =======================
    std_msgs::msg::Float64MultiArray action_msg;
    action_msg.data.resize(6);
    for (int i = 0; i < 6; ++i)
    {
        action_msg.data[i] = static_cast<double>(RLR.obs.action(i));
    }
    action_publisher_->publish(action_msg);
}

void MujocoSim::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr& msg)
{
    tracked_cmd_vel_x_ = static_cast<float>(msg->linear.x);
    tracked_cmd_vel_y_ = static_cast<float>(msg->linear.y);
    tracked_cmd_yaw_ = static_cast<float>(msg->angular.z);

    // teleop_twist_keyboard 기준:
    // linear.x : 전후진
    // linear.y : 좌우(스트레이프)
    // angular.z: 요 회전

    // 1) 입력을 [-1, 1] 범위로 정규화해서 RLR에 넣는 방식 권장
    //    (너 코드는 joy_cmd_x에 1.0*axes[1], joy_cmd_y에 0.75*axes[0], joy_cmd_z에 1.5*axes[3]를 기대)
    auto clamp = [](double v, double lo, double hi){
        return std::max(lo, std::min(hi, v));
    };

    // teleop에서 기본값이 보통 linear.x=0.5, angular.z=1.0 이런 식으로 나갈 수 있어서
    // 여기서 "최대 속도"를 정해놓고 나누어 [-1,1]로 만든다.
    const double MAX_LIN_X = 0.5;   // teleop의 speed 값과 맞추면 좋음
    const double MAX_LIN_Y = 0.5;   // (안 쓰면 0으로 둬도 됨)
    const double MAX_YAW_Z = 1.0;   // teleop의 turn 값과 맞추면 좋음

    double nx = (MAX_LIN_X > 1e-6) ? (msg->linear.x / MAX_LIN_X) : 0.0;
    double ny = (MAX_LIN_Y > 1e-6) ? (msg->linear.y / MAX_LIN_Y) : 0.0;
    double nz = (MAX_YAW_Z > 1e-6) ? (msg->angular.z / MAX_YAW_Z) : 0.0;

    nx = clamp(nx, -1.0, 1.0);
    ny = clamp(ny, -1.0, 1.0);
    nz = clamp(nz, -1.0, 1.0);

    // 2) 기존 조이스틱 로직이 하던 스케일을 그대로 적용
    RLR.obs.raw_joy_cmd_x = 1.0  * static_cast<float>(nx);
    RLR.obs.raw_joy_cmd_y = 0.75 * static_cast<float>(ny);
    RLR.obs.raw_joy_cmd_z = 1.5  * static_cast<float>(nz);

    RLR.obs.joy_cmd_x = RLR.obs.raw_joy_cmd_x;
    RLR.obs.joy_cmd_y = RLR.obs.raw_joy_cmd_y;
    RLR.obs.joy_cmd_z = RLR.obs.raw_joy_cmd_z;
}
