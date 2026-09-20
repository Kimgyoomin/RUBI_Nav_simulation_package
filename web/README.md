# RUBI Human Preference Path — browser pilot

RUBI의 두 경로를 실제 ONNX 정책과 MuJoCo WASM으로 계산하고, 동일 시간 배율로 재생한 뒤 경로 선호를 기록하는 웹 앱입니다. 기존 ROS 2/Gazebo 패키지와 독립적으로 실행됩니다.

**현재 상태: 모델을 연결하고 검증하는 draft pilot입니다.** 내부 XML, ONNX, STL은 포함하지 않습니다. 실제 RUBI의 평지·단차 보행 성공과 공개 설문의 중앙 저장 연결은 연구자의 실제 파일 및 서버로 확인해야 합니다. 파일 형식이 맞는 것과 보행이 성공하는 것은 별개의 검사입니다.

## 1. 로컬 실행

Node.js 24와 npm을 설치한 뒤 저장소 루트에서 실행합니다.

```bash
cd web
npm ci
npm run dev
```

브라우저에서 `http://127.0.0.1:5173`을 엽니다. `폴더 선택` 또는 `파일 선택`으로 아래 파일을 모두 불러옵니다. 파일 선택은 브라우저 메모리에서 처리하며 API로 업로드하지 않습니다. 새로고침 후에는 다시 선택해야 합니다.

| 파일 | 내용 |
| --- | --- |
| `rubi.xml` | MuJoCo MJCF, 링크 관성·관절·충돌·actuator·IMU 포함 |
| `encoder.onnx` | `mlp_input`, float32 `[330]` → `mlp_output`, float32 `[32]` |
| `policy.onnx` | `mlp_input`, float32 `[65]` → `mlp_output`, float32 `[6]` |
| `meshes/BODY.STL` | 몸체 메시 |
| `meshes/L_HIP.STL`, `L_THIGH.STL`, `L_CALF.STL`, `L_TIP.STL` | 왼쪽 다리 메시 |
| `meshes/R_HIP.STL`, `R_THIGH.STL`, `R_CALF.STL`, `R_TIP.STL` | 오른쪽 다리 메시 |

첨부 XML의 `compiler meshdir="meshes"` 기준입니다. 브라우저 파일 선택에서는 모두 같은 폴더에 있는 파일도 basename으로 연결합니다. 이름이 중복되면 임의로 선택하지 않고 거부합니다. URDF는 이 MJCF 경로에 추가로 필요하지 않습니다. ONNX external-data 파일을 사용하는 모델은 현재 지원하지 않으며, 두 ONNX에 가중치를 포함시켜 내보내야 합니다.

파일 유무만 먼저 확인하려면:

```bash
npm run inspect:bundle -- /absolute/path/to/private-rubi-bundle
```

Python 3 표준 라이브러리만 사용합니다. ONNX의 이름·형상·finite 출력은 브라우저에서 실제 세션을 생성하여 확인합니다. 현재 지원하는 것은 위 **rank-1 terrain 계약**이며, `[1,330]` 같은 임의의 batch 형상을 자동 변환하지 않습니다.

## 2. 어떤 제어 코드를 옮겼는가

기준은 저장소 commit `8991543654f2d608b9eb722906dd7f2740611ed6`의 다음 launch 경로입니다.

```bash
ros2 launch rubi_gazebo_sim rubi_gazebo_terrain_lidar.launch.py
```

이 경로에서 선택되는 `GazeboTerrainPolicyAdapter`의 계산을 TypeScript로 옮겼습니다. ROS 2와 Gazebo 자체를 브라우저에 로드하는 방식은 아닙니다. 웹에서는 MuJoCo WASM이 물리를, ONNX Runtime Web WASM이 신경망 추론을 처리합니다. 이 앱의 정해진 경로 추종에는 LiDAR, FastDEM, Nav2, A*가 필요하지 않습니다.

| 항목 | 웹 계약 |
| --- | --- |
| 물리 주기 | 0.002 s, 500 Hz |
| 정책 주기 | 물리 5 tick마다, 100 Hz |
| 관절 순서 | L_HR, L_HP, L_KN, R_HR, R_HP, R_KN |
| 기본 자세 | `[0, .65, -1.3, 0, .65, -1.3]` rad |
| PD | Kp 40, Kd 2, ±90 Nm |
| Actor 관측 | angular velocity 3, gravity 3, joint offsets 6, velocities 6, clipped previous action 6, phase 2, gait parameters 4, command 3 |
| History | 33 × 10 = 330, 오래된 프레임부터 최신 프레임 |
| Policy 입력 | encoder latent 32 다음 actor 33 |
| IMU | MuJoCo WXYZ quaternion → adapter XYZW; gyro local frame |

원본 C++ 실행에서 생성한 59개 snapshot을 대상으로 28,497개 스칼라 값을 비교하는 회귀 테스트가 포함됩니다. 이는 **제어 수학의 비교**이며 실제 ONNX 모델, 전체 Gazebo 플러그인, 물리 접촉의 동등성을 증명하지 않습니다.

다음 차이는 실제 모델 검증 때 확인해야 합니다.

- 원본 Gazebo 플러그인의 센서 sample/hold, 한 주기 지연, IMU noise와 웹의 직접 MuJoCo 상태 읽기는 같지 않습니다.
- MuJoCo와 Gazebo의 접촉, 마찰, solver, inertia 해석이 다를 수 있습니다.
- 웹은 경로마다 physics/history를 초기화합니다. body/world weld가 있으면 자세 준비 동안 유지하고, 252 ready tick 후 해당 weld만 해제합니다.
- 첨부 XML은 freejoint `root`와 6개 hinge, 직접 torque motor, `imu_quat`와 `angular_velocity` sensor를 요구합니다. 모델의 knee 초기 상태와 ready 전환이 이 backend에서 안정적인지는 실제 메시·정책으로 확인해야 합니다.
- 원본 action clip/PD 계산과 별도로, 추론 오류가 발생하면 물리 반복을 중지합니다. C++ emergency-stop latch 전체를 이식한 것은 아닙니다.

## 3. XML과 지형 처리

원본 파일을 변경하지 않고 브라우저 메모리의 XML을 수정합니다. 루트의 world/terrain/stair include를 제거하고 설문용 플랫폼을 추가합니다. 현재 첨부된 `lrc_flat_hfield_world.xml` include는 자동 교체되므로 수동 주석 처리는 선택 사항입니다. 로봇 내부 include가 있다면 하나의 MJCF로 합쳐야 합니다.

로봇의 inertial, actuator와 충돌 geom은 보존합니다. **L_TIP/R_TIP 메시도 실제 충돌에 사용되므로 STL을 전부 지우고 동일 보행이라고 취급하면 안 됩니다.** BODY에 연결된 시작 지지 weld는 자동 인식해 준비 완료 후 해제합니다. 다른 equality constraint는 해제하지 않습니다.

현재 장면은 동일 출발점·목적점, 6 m 직진, 길이 0.8 m × 폭 0.7 m 플랫폼입니다. 단차 높이 0–12 cm, 추가 우회거리 0.4–2.4 m, 전진 속도 명령 0.1–0.5 m/s를 조절합니다. 이 범위는 UI의 입력 범위이며 RUBI의 통과 가능 범위가 아닙니다.

두 경로는 정책으로 물리 실행한 뒤 qpos를 기록해 1×로 재생합니다. 경로 위로 로봇을 강제 이동시키거나 걷는 애니메이션을 합성하지 않습니다. 넘어짐·경로 이탈·시간 초과가 발생하면 선호 응답을 잠급니다. 모든 조사 조건은 배포 전에 실제 모델로 완주를 확인하세요.

## 4. GitHub Pages 배포

`.github/workflows/human-preference-web.yml`이 테스트·빌드와 Pages artifact 생성을 수행합니다. 내부 모델은 workflow에 필요하지 않습니다.

1. 저장소 **Settings → Pages → Build and deployment → Source → GitHub Actions**를 선택합니다.
2. **Actions → Human Preference Web**에서 실행을 재실행하거나 `main`의 웹 파일 변경을 push합니다.
3. `deploy` job이 성공하면 GitHub가 표시하는 실제 Pages 주소로 접속합니다.

기본 주소는 `https://kimgyoomin.github.io/RUBI_Nav_simulation_package/`입니다. 실제 publish 성공 전에는 이 주소에서 서비스가 열린다고 보장하지 않습니다. `has_pages=false`이면 workflow는 빌드 artifact만 생성하고 배포를 건너뜁니다. `configure-pages`는 기본 GITHUB_TOKEN만으로 최초 Pages 활성화를 대신하지 않습니다.

`feature/human-preference-web`에서도 빌드할 수 있습니다. 해당 branch를 직접 배포하려면 `github-pages` environment의 배포 branch 정책이 허용해야 합니다. 일반 운영은 `main`을 사용하세요. workflow 수정 권한이 없는 token으로 push하면 GitHub가 workflow 파일 쓰기를 거부할 수 있습니다.

GitHub Pages는 정적 호스팅이므로 Node API나 SQLite를 실행하지 않습니다. 브라우저는 single-thread WASM 경로를 사용하며 SharedArrayBuffer/COOP/COEP 설정을 요구하지 않습니다. 페이지 주소의 repository 하위 경로와 WASM 자산 경로는 Vite 빌드에서 설정합니다.

공식 문서: [GitHub Pages custom workflows](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages), [MuJoCo WASM](https://github.com/google-deepmind/mujoco/tree/main/wasm), [ONNX Runtime Web](https://onnxruntime.ai/docs/tutorials/web/).

## 5. 링크만으로 누구나 참여시키려면

기본 배포는 **연구자 준비 화면**이며 참가자에게 내부 파일을 요청하는 공개 설문으로 운영하면 안 됩니다. 링크만으로 참여시키려면 연구자가 모델 배포와 응답 저장을 먼저 설정해야 합니다.

- 모델 배포를 허용할 수 있다면 `public/study.json`의 `bundleBaseUrl`에 XML·ONNX·STL을 제공하는 폴더 URL을 넣습니다. 브라우저가 내려받는 파일은 참가자가 추출할 수 있습니다. private GitHub repository나 난독화로 이를 숨길 수는 없습니다. 다른 origin의 자산 서버는 CORS를 설정해야 합니다. URL은 `/`로 끝나야 합니다.
- 내부 모델 배포를 허용할 수 없다면, 연구실에서 정책을 실행하고 **검증된 영상 또는 공개 가능한 trajectory/시각 모델**을 배포하는 별도 경로가 필요합니다. 현재 앱은 로컬 private file loading과 hosted bundle을 구현하며, 서버 원격 시뮬레이션/영상 전용 설문은 포함하지 않습니다.
- `responseApi`는 실제 HTTPS 응답 서버의 `/api` 주소로 설정합니다. 모델 자산과 중앙 저장을 확인하고 장면 검증을 마친 뒤 `status`를 `released`로 바꿉니다. 서버의 연구 ID 및 상태도 같아야 합니다.

연구 설정을 바꾸면 `id`도 버전 증가시켜 서로 다른 조건을 같은 자료로 섞지 마세요. 기본 설정의 높이와 거리 조합은 화면을 점검할 pilot 예시입니다.

## 6. 응답 API

로컬 개발에서 두 번째 터미널을 열고 실행합니다.

```bash
cd web
npm run api
```

앱의 `응답 저장 연결`에 `http://127.0.0.1:8787/api`를 넣고 연결을 확인합니다. 또는 `.env.local`에 `VITE_RESPONSE_API`를 지정합니다. Vite는 local `.env`를 읽지만 Node API 환경 변수는 실행 환경이나 `node --env-file=.env.local api/server.mjs`로 별도 전달해야 합니다.

| API 환경 변수 | 의미 |
| --- | --- |
| `RUBI_API_PORT` | 기본 8787 |
| `RUBI_DB_PATH` | 기본 `data/responses.sqlite`; 배포 시 persistent volume 경로 사용 |
| `RUBI_ALLOWED_ORIGIN` | 허용 웹 origin의 comma 목록. Pages는 `https://kimgyoomin.github.io` |
| `RUBI_ADMIN_TOKEN` | export용 비밀 토큰. **VITE_ 변수에 넣거나 웹 코드에 배포하지 않음** |
| `RUBI_EXPERIMENT_ID` | 기본 `rubi-hpp-pilot-v1` |
| `RUBI_STUDY_STATUS` | 기본 `draft`; 공개 설문은 `released` |

HTTPS와 지속 디스크를 지원하는 연구실 서버 또는 컨테이너 호스트에서 운영합니다. Docker build context는 `web`입니다.

```bash
docker build -f api/Dockerfile -t rubi-hpp-api .
```

`GET /api/health`, `POST /api/responses`, `GET /api/export`, `GET /api/export?format=csv`를 제공합니다. Export에는 `Authorization: Bearer <RUBI_ADMIN_TOKEN>`을 보냅니다. POST는 두 경로의 완주 metadata, 시청 범위, 참여 동의, 모델 hash, 설정 범위를 검증합니다. 동일 submission ID의 재시도는 멱등 처리하고 participant/experiment/trial 중복은 거부합니다. 모델 원본과 trajectory frames는 응답 API로 전송하지 않습니다.

API는 JSON과 요약 CSV를 제공합니다. 모델 hash, 자세한 장면·회전·시청 event 분석에는 JSON을 보관하세요. 브라우저 미리보기의 기기 저장은 중앙 수집이 아니며 UI에 이를 구분해서 표시합니다. 브라우저 UUID는 동일 기기의 반복 trial을 묶는 키이며 실제 사람을 유일하게 식별하지 않습니다. 이 API는 pilot 수집용으로, 웹 클라이언트가 보낸 metadata가 실제 보행 결과인지 암호학적으로 검증하지 않습니다.

## 7. 설문과 cost 해석

질문은 **“RUBI에게 어느 경로를 지정하시겠습니까?”**입니다. 응답은 경로 선호이며 안전확률 또는 물리적 에너지 정답이 아닙니다. 참가자별 trial 순서와 A/B 매핑을 고정 무작위화하고, 두 보행의 시청 범위를 확인한 후 선택을 엽니다. 판단 지연은 두 영상 확인 시점 이후부터, 전체 trial 소요시간은 별도로 기록합니다. `판단 어려움`을 임의의 A/B label로 바꾸지 않습니다.

현재 플랫폼은 **상승 + 하강**을 포함하므로 단일 상승 mechanical work와 직접 같다고 비교하지 않습니다. 우회거리 변화는 회전 각도와도 연결됩니다. 현재 자료만으로 높이의 순수 효과와 회전 효과를 독립 추정할 수 있다고 주장하지 마세요. 회전 각도·초기 회전·계획거리·실제 주행거리·주행시간을 기록하며, 본 조사에는 회전 조건을 별도로 통제한 장면군이 필요합니다.

예를 들어 `P(direct)=sigmoid(beta0 + betaL * deltaL - f(height))`를 사용하면, `betaL>0`이고 실제 관측 범위 안에서 경계가 추정되는 경우 `C_H(h)=(f(h)-beta0)/betaL`입니다. A/B presentation, 참가자 반복측정과 불확실성도 분석에 포함해야 합니다. 이 앱은 자료 수집과 export를 제공하며 추정값을 임의로 만들거나 planner에 자동 반영하지 않습니다.

## 8. 코드와 검증

| 경로 | 역할 |
| --- | --- |
| `src/core/terrain-controller.ts` | terrain 관측, encoder/actor 계약, history, PD |
| `src/core/model.ts` | 로컬 파일 연결, hash, MJCF 장면 교체 |
| `src/core/scenario.ts` | 지형·경로 조건과 경로 추종 |
| `src/core/study.ts` | 중복 없는 시청 범위, trial 무작위화 |
| `src/runtime/physics.ts` | MuJoCo 초기화, physics rollout, 기록 재생 |
| `src/runtime/onnx.ts` | ONNX 세션·shape·수치 검사 |
| `src/runtime/viewer.ts` | Three.js 3D 지형과 실제 모델 렌더링 |
| `src/main.ts` | 연구자 화면, 설문 진행, 응답 전송 |
| `api/server.mjs` | SQLite 영구 저장, 검증, 중복 방지, export |
| `public/study.json` | 연구 ID, 공개 상태, 파일/API URL, 장면 목록 |

```bash
npm test
npm run build
```

테스트는 C++ fixture 비교, 실제 SQLite HTTP 저장/재시작/중복/권한, 경로 거리와 시청 범위를 확인합니다. `tests/fixtures/terrain_contract_probe.cpp`는 원본 C++에서 golden 값을 생성한 코드이며 실제 ONNX를 포함하지 않습니다. 모델이 없는 상태에서 테스트 통과를 실제 RUBI locomotion 성공이라고 해석하지 마세요.
