/**
 * @file rclab_mujoco_sim_main.h
 * @author YunHo Han (93yunho@gmail.com), Jeong-Hwan Jang(jang990608@gmail.com)
 * @brief MuJoCo 시뮬레이션을 위한 메인 ROS 1 노드 클래스를 정의하는 헤더 파일입니다.
 * @version 0.3.0
 * @date 2025-10-13
 * @copyright Copyright (c) 2025
 */
#ifndef RCLAB_MUJOCO_SIM_MAIN_H
#define RCLAB_MUJOCO_SIM_MAIN_H

// --- C++ Standard Libraries ---
#include <chrono>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <Eigen/Dense>
// keyboard
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <std_msgs/msg/float64_multi_array.hpp>

// --- Third-party Libraries ---
#include <mujoco/mujoco.h>  // MuJoCo 물리 시뮬레이션 라이브러리

// --- ROS 2 Libraries ---
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

// --- Custom Project Headers ---
#include "rclab_mujoco_sim/rclab_mujoco_ui.h"  // 시뮬레이션 UI 관련 클래스
#include "RLRobot/RLRobot.h"
#include "onnx_wrap/onnx_wrap.h" 


/**
 * @class MujocoSim
 * @brief MuJoCo 시뮬레이션을 관리하고 ROS 1과 연동하는 메인 클래스입니다.
 */
class MujocoSim {
   public:
    /**
     * @brief MujocoSim 클래스의 생성자입니다.
     * 노드 핸들 설정, 모델 로드, 퍼블리셔 생성 등 기본적인 초기화를 수행합니다.
     */
    explicit MujocoSim(const rclcpp::Node::SharedPtr& node);

    /**
     * @brief MujocoSim 클래스의 소멸자입니다.
     * 객체 소멸 시 할당된 모든 리소스(MuJoCo, GLFW, ImGui)를 안전하게 해제합니다.
     */
    ~MujocoSim();

    /**
     * @brief 시뮬레이션 환경의 모든 상세 초기화를 담당하는 함수입니다.
     * GLFW 윈도우 생성, ImGui 설정, MuJoCo 렌더링 컨텍스트 및 콜백 함수 설정을 포함합니다.
     * @return true 초기화 성공 시
     * @return false 초기화 실패 시
     */
    bool init();

    /**
     * @brief 시뮬레이션의 메인 루프를 실행하는 함수입니다.
     * 이 루프는 ROS가 종료되거나 사용자가 창을 닫을 때까지 계속 실행됩니다.
     */
    void run();

   private:
    struct SimulationSnapshot;
    /**
     * @brief 현재 로봇의 모든 관절 상태(위치, 속도)를 ROS 1 토픽으로 발행합니다.
     */
    void publish_joint_states();

    /**
     * @brief MuJoCo 데이터로부터 현재 액추에이터가 제어하는 관절의 상태(위치, 속도)를 읽어와 멤버 변수에 업데이트합니다.
     */
    void update_joint_state();

    /**
     * @brief PD 제어 법칙에 따라 목표 자세를 유지하기 위한 토크를 계산하고 시뮬레이션에 적용합니다.
     */
    void apply_pd_control();

    /**
     * @brief [추가] 시뮬레이션의 한 스텝을 진행시키는 헬퍼 함수입니다.
     * @note 이 함수는 상태 업데이트, 제어 적용, 외란 적용, 물리 계산의 한 단위를 캡슐화하여 코드 중복을 방지합니다.
     */
    void advance_simulation();
    void reset_to_startup_state();

    // --- Reinforcement Learning Control Functions --- //
    void INIT_Networks_ALL();      // [수정] 모든 네트워크 한 번에 로드
    void Switch_To_Walk_Mode();    // [추가] 모드 전환 및 버퍼 마이그레이션 함수
    void INIT_Network();
    void INIT_Network3();
    void Inference3(); // Walking (30 obs)
    void JointPDTorqueControl_RL(); // by gemini
    void ROSJoyMode(const sensor_msgs::msg::Joy::SharedPtr& msg);
    void IMUSensorRead(); 
    void EncoderRead(); 
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr& msg);
    void configure_state_history();
    void clear_state_history();
    void trim_future_state_history();
    void capture_current_state();
    void restore_snapshot(const SimulationSnapshot& snapshot);
    void restore_state_from_history_index(size_t index);
    void step_paused_history(double duration_seconds, int direction);
    int step_count_from_duration(double duration_seconds) const;

    // --- MuJoCo Core Variables ---
    mjModel* model_ = nullptr;      // 시뮬레이션의 모든 정적 데이터(물리 특성, 지오메트리 등)를 담는 구조체
    mjData* data_ = nullptr;        // 시뮬레이션의 모든 동적 데이터(위치, 속도, 힘 등)를 담는 구조체
    GLFWwindow* window_ = nullptr;  // 렌더링을 위한 GLFW 윈도우 포인터
    char error_[1000] = {0};        // MuJoCo API 에러 메시지를 저장하기 위한 버퍼

    // --- ROS 2 Communication ---
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
    float tracked_cmd_vel_x_ = 0.0f;
    float tracked_cmd_vel_y_ = 0.0f;
    float tracked_cmd_yaw_ = 0.0f;
    std::vector<std::string> urdf_joint_names_;                                         // ROS 메시지에 사용할 관절 이름 목록
    std::vector<int> urdf_joint_qpos_indices_;                                          // `data_->qpos`에서 관절 위치를 찾기 위한 인덱스 목록
    std::vector<int> urdf_joint_qvel_indices_;                                          // `data_->qvel`에서 관절 속도를 찾기 위한 인덱스 목록
    std::map<int, int> joint_dof_to_actuator_map_;                                      // 각 관절의 DOF를 액추에이터 인덱스에 매핑하는 맵

    // --- UI and Callbacks ---
    rclab_mujoco_ui rclab_ui_;            // UI 관리를 위한 클래스 인스턴스
    CallbackUserPointers user_pointers_;  // GLFW 콜백 함수에 model, data, ui 포인터를 전달하기 위한 구조체

    // --- PD Control ---
    std::vector<int> actuator_qpos_adr_;          // 각 액추에이터에 연결된 관절의 `qpos` 주소
    std::vector<int> actuator_qvel_adr_;          // 각 액추에이터에 연결된 관절의 `qvel` 주소
    std::vector<double> current_qpos_;            // 현재 관절 위치를 저장하는 버퍼
    std::vector<double> current_qvel_;            // 현재 관절 속도를 저장하는 버퍼
    std::vector<double> current_actuator_force_;  // 이전 스텝에서 계산된 액추에이터 힘을 저장하는 버퍼
    std::vector<double> actuator_ctrl_min_;       // 액추에이터 제어의 최소값
    std::vector<double> actuator_ctrl_max_;       // 액추에이터 제어의 최대값

    std::vector<double> pd_desired_pos_;  // 목표 위치를 저장하는 버퍼
    std::vector<double> pd_kp_;           // 비례 게인(목표 위치와 현재 위치의 차이에 대한 반응)을 저장하는 버퍼
    std::vector<double> pd_kd_;           // 미분 게인(속도 변화에 대한 반응)을 저장하는 버퍼
    std::vector<double> pd_torque_;       // 계산된 토크를 저장하는 버퍼

    // --- RL Control --- // by gemini
    RLRobot RLR; // by gemini
    ONNX onnx; // by gemini
    ONNX onnx_encoder;
    ONNX onnx_policy;
    ONNX onnx_encoder_stand;
    ONNX onnx_policy_stand;
    ONNX onnx_encoder_walk;
    ONNX onnx_policy_walk;
    int decimation = 0; // by gemini
    bool emergency_stop = false; // by gemini
    // -------------------------------------
    // 관측(Observation) 버퍼
    int obsHistoryLength_ = 20;
    int observationSize_ = 30; // rubi_config.py 기준 (BaseAngVel(3) + Gravity(3) + DofPos(6) + DofVel(6) + Actions(6) + Clock(2) + Gait(4) = 30)
    int commandSize_ = 3;
    int actorObsSize_ = 33;       // CTS actor obs = Obs(30) + Commands(3)
    int encoderOutputSize_ = 32;  // CTS student encoder output = latent 32
    int encoderInputSize_ = actorObsSize_ * obsHistoryLength_;  // 33 * 10 = 330
    int policyInputSize_ = encoderOutputSize_ + actorObsSize_;  // 32 + 33 = 65
    bool obs_history_initialized_ = false;

    // [추가] 모드 및 상태 관리 변수
    bool is_walking_mode = false;      // 현재 걷기 모드인지 여부
    double policy_start_time = 0.0;    // POLICY_ON 시작 시간 기록용
    double current_gait_index_ = 0.0;  // 보행 위상 (Phase)

    // obs_history를 저장할 큐 (Eigen::VectorXf를 저장)
    std::deque<Eigen::VectorXf> obsHistoryQueue_;

    // 추론용 벡터
    Eigen::VectorXf currentObs_;
    Eigen::VectorXf currentActorObs_;
    Eigen::VectorXf encoderOutput_;
    Eigen::VectorXf policyInput_;
    Eigen::VectorXf obsHistoryFlat_; // 1D로 펼쳐진 obs_history

    struct SimulationSnapshot {
        std::vector<mjtNum> mj_state;
        rclab_mujoco_ui::VelocityHistoryState velocity_history;
        RLRobot rlr_state;
        std::deque<Eigen::VectorXf> obs_history_queue;
        Eigen::VectorXf current_obs;
        Eigen::VectorXf current_actor_obs;
        Eigen::VectorXf encoder_output;
        Eigen::VectorXf policy_input;
        Eigen::VectorXf obs_history_flat;
        int decimation = 0;
        bool emergency_stop = false;
        bool obs_history_initialized = false;
        bool is_walking_mode = false;
        double policy_start_time = 0.0;
        double current_gait_index = 0.0;
        float tracked_cmd_vel_x = 0.0f;
        float tracked_cmd_vel_y = 0.0f;
        float tracked_cmd_yaw = 0.0f;
    };
    std::deque<SimulationSnapshot> state_history_;
    SimulationSnapshot pause_anchor_snapshot_;
    size_t history_cursor_ = 0;
    size_t max_history_size_ = 0;
    int integration_state_size_ = 0;
    bool pause_anchor_valid_ = false;
    size_t pause_anchor_cursor_ = 0;
    bool was_paused_last_frame_ = false;

};

#endif  // RCLAB_MUJOCO_SIM_MAIN_H
