#include <atomic>
#include <cassert>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <limits>
#include <mutex>
#include <thread>
#include "task.h"
#include "tracking_mode/tracking_control/tracking_mode.hpp"
#include "tracking_mode/tracking_generation/trajectory_runtime.hpp"
#include "motor/rm_motor/rm_motor.hpp"
using namespace tracking;
using namespace tracking_config;
namespace {
std::atomic<uint32_t> clock_ms{0};
std::mutex gate;
std::condition_variable cv;
unsigned yields=0;
bool resume=false,stop=false;
struct Stop {};
}
TickType_t xTaskGetTickCount() { return clock_ms; }
void vTaskDelayUntil(TickType_t *previous,TickType_t increment) {
  *previous+=increment;
  std::unique_lock<std::mutex> lock(gate); ++yields; cv.notify_all();
  cv.wait(lock,[]{return resume||stop;});
  if (stop) throw Stop{};
  resume=false;
}
struct Scheduler {
  std::thread worker;
  Scheduler() : worker([]{ try { trajectory_task(nullptr); } catch (Stop &) {} }) {
    std::unique_lock<std::mutex> lock(gate); cv.wait(lock,[]{return yields>0;});
  }
  void tick(uint32_t delta=1,bool publish=true) {
    std::unique_lock<std::mutex> lock(gate); clock_ms+=delta;
    if (!publish) return;
    const auto old=yields; resume=true; cv.notify_all();
    cv.wait(lock,[old]{return yields>old;});
  }
  ~Scheduler() {
    { std::lock_guard<std::mutex> lock(gate); stop=true; cv.notify_all(); }
    worker.join();
  }
};
bool near(float a,float b,float eps=1e-5f) { return std::fabs(a-b)<eps; }
void zero(Command c) { assert(c.torque_a==0&&c.torque_b==0&&c.torque_c==0); }
void controller_tests() {
  auto g=kYaw; g.angle_kp=0; g.speed_kp=0; g.speed_ki=0;
  g.inertia_ff=.1f; g.damping_ff=.2f; g.max_torque=100; g.max_acceleration=1000;
  g.acceleration_filter_s=0;
  AxisController c(g);
  auto a=c.step({0,1,999},0,0,.001f);
  assert(near(a.feedforward_torque,.2f)); // no first-cycle derivative spike
  a=c.step({0,1.1f,-999},0,0,.001f);
  assert(near(a.acceleration_command,100,.01f));
  assert(near(a.feedforward_torque,10.22f,.002f)); // d(speed command), NOT r.acceleration
  c.reset(); a=c.step({0,1.1f,0},0,0,.001f); assert(a.acceleration_command==0);
  g=kYaw; g.angle_kp=0; g.speed_kp=1; g.speed_ki=10; g.max_torque=.3f;
  AxisController pi(g);
  for (int i=0;i<1000;++i) { a=pi.step({0,1,0},0,0,.001f); assert(a.torque<=.3f); }
  assert(pi.integral()==0); // conditional integration includes final saturation
  for (int i=0;i<100;++i) pi.step({0,.01f,0},0,0,.001f);
  assert(pi.integral()>0);
  pi.step({0,0,0},std::numeric_limits<float>::quiet_NaN(),0,.001f);
  assert(pi.integral()==0);
  a=pi.step({10,10,0},0,0,.001f,.5f); assert(a.speed_command==.5f&&a.saturated);
  assert(near(approach(1,2,0,2).position,1));
  assert(near(approach(1,2,2,2).position,2)); assert(approach(1,2,2,2).velocity==0);
  std::puts("PASS controller: PDF J*d(speed_set)/dt+b*speed_set, reset, limits, antiwindup");
}
void trajectory_tests() {
  float peak_y=0,peak_p=0;
  for (uint32_t i=0;i<TRAJECTORY_PERIOD_MS;++i) {
    auto p=trajectory_sample_at_ms(i),repeat=trajectory_sample_at_ms(i+TRAJECTORY_PERIOD_MS);
    assert(near(p.yaw,repeat.yaw)&&near(p.pitch,repeat.pitch));
    assert(p.yaw>=kYawMin&&p.yaw<=kYawMax&&p.pitch>=kPitchMin&&p.pitch<=kPitchMax);
    assert(std::fabs(p.yaw_acc)<=110.01f&&std::fabs(p.pitch_acc)<=70.01f);
    peak_y=std::max(peak_y,std::fabs(p.yaw_vel)); peak_p=std::max(peak_p,std::fabs(p.pitch_vel));
  }
  std::printf("PASS actual trajectory: 680 ms, peaks %.6f/%.6f rad/s\n",peak_y,peak_p);
}
void motor_tests() {
  sp::RM_Motor a(1,sp::RM_Motors::GM6020),b(2,sp::RM_Motors::GM6020);
  sp::RM_Motor c(3,sp::RM_Motors::M2006,36,true);
  assert(a.rx_id==0x205&&b.rx_id==0x206&&c.rx_id==0x203);
  assert(a.tx_id==0x1fe&&b.tx_id==0x1fe&&c.tx_id==0x200);
  uint8_t data[8]={0x10,0,0x08,0x70,0,0,25,0}; // 2160 rotor rpm = 2*pi output rad/s
  c.read(data,10); assert(near(c.speed,6.2831853f)); assert(c.is_alive(109)&&!c.is_alive(110));
  uint8_t out[8]={}; c.cmd(.18f); c.write(out);
  const int raw=(out[4]<<8)|out[5]; assert(raw>=999&&raw<=1000);
  assert(out[0]==0&&out[1]==0&&out[2]==0&&out[3]==0&&out[6]==0&&out[7]==0);
  data[0]=0x1f;data[1]=0xf0; c.read(data,11);
  const float before=c.angle; data[0]=0;data[1]=0x10;c.read(data,12);
  assert(std::fabs(c.angle-before)<.01f); // encoder wrap divided by 36
  std::puts("PASS actual RM middleware: IDs, frame slots, reduction, torque, rollover");
}
struct Rig {
  Scheduler &s;
  Feedback f{};
  float initial_pitch=.25f,initial_c=-.3f,initial_b=2.3f;
  explicit Rig(Scheduler &scheduler):s(scheduler) {}
  Command call(bool request=true,bool down=false,bool remote=true,uint32_t gap=1,bool producer=true) {
    s.tick(gap,producer); f.imu_stamp_ms=clock_ms;
    return update(request,down,remote,f,clock_ms,gap);
  }
  void start() {
    f={}; f.angle_a=1.2f; f.angle_b=initial_b; f.angle_c=initial_c;
    f.pitch=initial_pitch; f.motors_alive=f.imu_ready=true;
    zero(call(false,true)); call(); assert(read_telemetry().state==State::WaitImu);
  }
  void follow() { // KINEMATIC ideal feedback, not a motor dynamics simulation.
    auto t=read_telemetry();
    if (t.state==State::WaitImu) return;
    const float pitch=t.target_pitch+t.pitch_origin;
    const float angle=t.yaw_origin+t.target_yaw/kYawDirection;
    f.pitch_rate=(pitch-f.pitch)*1000; f.speed_c=f.pitch_rate/kTipDirection;
    f.speed_a=(angle-f.angle_a)*1000;
    f.angle_a=angle; f.pitch=pitch;
    f.angle_c=initial_c+(pitch-initial_pitch)/kTipDirection;
  }
  void to(State state) {
    for (unsigned i=0;i<16000&&read_telemetry().state!=state;++i) {
      follow(); call();
      const auto t=read_telemetry();
      if (t.state==State::Fault) std::printf("unexpected fault %u while reaching %u\n",unsigned(t.fault),unsigned(state));
      assert(t.state!=State::Fault); assert(near(t.arm_hold,initial_b));
    }
    assert(read_telemetry().state==state);
  }
  void expect(Fault fault,Command c) {
    zero(c); assert(read_telemetry().state==State::Fault);
    if (read_telemetry().fault!=fault) std::printf("fault %u expected %u\n",unsigned(read_telemetry().fault),unsigned(fault));
    assert(read_telemetry().fault==fault);
  }
};
void flow_tests(Scheduler &s) {
  Rig r(s); r.start(); r.to(State::Align);
  auto t=read_telemetry(); assert(t.calibrated&&std::fabs(t.pitch_origin)<.01f);
  assert(near(t.arm_hold,2.3f)); assert(std::fabs(t.actual_pitch)<1e-5f);
  r.to(State::Running); assert(read_telemetry().phase_ms==0);
  for (int i=0;i<1360;++i) { r.follow(); r.call(); assert(read_telemetry().state==State::Running); }
  t=read_telemetry(); assert(t.samples>1300&&t.rms_yaw<.02f&&t.rms_pitch<.02f);
  assert(t.peak_error_yaw>=t.rms_yaw); assert(near(t.arm_hold,2.3f));
  r.expect(Fault::RemoteLost,r.call(true,false,false));
  zero(r.call()); assert(read_telemetry().fault==Fault::RemoteLost); // latched
  r.start(); assert(!read_telemetry().calibrated); // restart recalibrates, no stale origin
  r.to(State::Level); r.f.imu_ready=false; r.expect(Fault::ImuLost,r.call());
  r.start(); r.f.motors_alive=false; r.expect(Fault::FeedbackLost,r.call());
  r.start(); r.expect(Fault::Timing,r.call(true,false,true,6));
  r.start(); r.f.pitch=std::numeric_limits<float>::quiet_NaN(); r.expect(Fault::NonFinite,r.call());
  r.start(); r.f.roll=kMaxRoll+.1f; r.expect(Fault::AttitudeLimit,r.call());
  r.start(); r.f.angle_c+=kMaxLevelTravel+.1f; r.expect(Fault::TravelLimit,r.call());
  r.start(); r.f.angle_b+=.2f;
  for (unsigned i=0;i<kErrorDurationMs;++i) r.call();
  r.expect(Fault::ArmMoved,r.call());
  r.start(); r.f.imu_ready=false;
  for (unsigned i=0;i<kWaitImuTimeoutMs;++i) r.call();
  r.expect(Fault::ImuWaitTimeout,r.call());
  r.start(); r.to(State::Level); // deliberately refuse to move C
  for (unsigned i=0;i<kLevelTimeoutMs;++i) r.call();
  r.expect(Fault::LevelTimeout,r.call());
  r.start(); r.to(State::Align);
  for (int i=0;i<10;++i) r.call(true,false,true,1,false);
  r.expect(Fault::ReferenceStale,r.call(true,false,true,1,false));
  r.start(); r.to(State::Align);
  for (unsigned i=0;i<kAlignTimeoutMs;++i) r.call();
  r.expect(Fault::AlignTimeout,r.call());
  r.start(); r.to(State::Running);
  for (unsigned i=0;i<=kErrorDurationMs;++i) {
    s.tick(); const auto p=trajectory_snapshot().point;
    r.f.imu_stamp_ms=clock_ms; r.f.pitch=p.pitch;
    r.f.angle_a=read_telemetry().yaw_origin+
      (p.yaw>.0f?p.yaw-(kMaxTrackingError+.01f):p.yaw+(kMaxTrackingError+.01f))/kYawDirection;
    auto c=update(true,false,true,r.f,clock_ms,1);
    if (i==kErrorDurationMs) r.expect(Fault::TrackingError,c);
  }
  // Tick overflow while startup and trajectory task are both active.
  clock_ms=UINT32_MAX-250; r.start(); r.to(State::Running);
  assert(clock_ms<20000); assert(read_telemetry().phase_ms==0);
  zero(r.call(false,true));
  std::puts("PASS full state sequence, B hold, origin, restart, RMS and tick wrap");
  std::puts("PASS motor/IMU/remote loss, NaN, attitude/travel, timing, B drift, all stage timeouts, stale trajectory, tracking error");
}
int main() {
  controller_tests(); trajectory_tests(); motor_tests();
  Scheduler s; flow_tests(s);
  std::puts("All host tests passed. No hardware dynamics, ISR timing or physical stability claim.");
}
