#include <string>
#include <thread>
#include <vector>
#include <random>
#include <ostream>
#include <iostream>
#include <sstream>
#include <algorithm>


class PidConf
{
public:
  PidConf(double kp, double ki, double kd, double i_limit = 50)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , i_limit_(i_limit)
  {}

  PidConf(const PidConf& pidconf)
    : PidConf(pidconf.kp_, pidconf.ki_, pidconf.kd_, pidconf.i_limit_)
  {}

  void SetKp(double kp)
  {
    kp_ = kp;
  }
  void SetKi(double ki)
  {
    ki_ = ki;
  }
  void SetKd(double kd)
  {
    kd_ = kd;
  }

  double GetKp() const
  {
    return kp_;
  }
  double GetKi() const
  {
    return ki_;
  }
  double GetKd() const
  {
    return kd_;
  }

private:
  double kp_;
  double ki_;
  double kd_;

  double i_limit_;
};


class PidControl
{
public:
  PidControl(const PidConf& pid_conf): pid_conf_(pid_conf) {}

  double Control(double error)
  {
    ki_all_ += error;
    ki_all_ = std::min(100., ki_all_);
    return -pid_conf_.GetKp() * error -pid_conf_.GetKi() * ki_all_ +
           pid_conf_.GetKd() * error;
  }

  void SetKp(double kp)
  {
    pid_conf_.SetKp(kp);
  }
  void SetKi(double ki)
  {
    pid_conf_.SetKi(ki);
  }
  void SetKd(double kd)
  {
    pid_conf_.SetKd(kd);
  }

  double GetKp() const
  {
    return pid_conf_.GetKp();
  }
  double GetKi() const
  {
    return pid_conf_.GetKi();
  }
  double GetKd() const
  {
    return pid_conf_.GetKd();
  }

private:
  PidConf pid_conf_;
  double ki_all_{0};
};


int main()
{
  std::cout << "当前水温：" << 0 << "  目标水温：" << 100;
  double temple = 0.;

  PidConf pidconf(0.5, 0.5, 0.5);
  PidControl pid(pidconf);
  while(true)
  {
    // std::this_thread::sleep_for(std::chrono::duration<std::chrono::seconds, std::ratio<1, 1>>(1));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "当前的 temple: " << temple << "\n";
    temple = pid.Control(temple - 100);
  }
  return 0;
}