#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <sstream>
#include <condition_variable>


#include <cstdio>
#include <ctime>


class Shared
{
 private:
  std::mutex mtx_;
  std::condition_variable cond_;
  int count;
  
 public:
  Shared() : count(0) {}
  ~Shared() {}

  void signal(void)
  {
    std::lock_guard<std::mutex> lg(mtx_);
    count++;
    cond_.notify_one();
  }
  
  void wait_one(void)
  {
    std::unique_lock<std::mutex> lg(mtx_);
    while(count==0) { cond_.wait(lg); }
    count--;
  }
  
};


void dump_clock(const char *mesg)
{
  std::stringstream thrid_ss;
  thrid_ss << std::this_thread::get_id();
  
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tm;
  gmtime_r(&(ts.tv_sec), &tm);
  char buf1[64];
  strftime(buf1, 63, "-- %Y%m%dT%H%M%S.%%06dZ 0x%%s: %%s", &tm);
  const int size=128;
  char buf[size];
  std::snprintf(buf, size, buf1, ts.tv_nsec/1000, thrid_ss.str().c_str(), mesg);
  std::cout << buf << std::endl;
}

void producer(Shared &shared)
{
  dump_clock("pp1");
  for(int idx=0; idx<10; idx++) {
    shared.signal();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    dump_clock("pp2");
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));
  dump_clock("pp3");

  for(int idx=0; idx<10; idx++) {
    shared.signal();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    dump_clock("pp4");
  }
}


int main(void)
{
  Shared shared;

  std::thread prod(producer, std::ref(shared));

  dump_clock("cp1");
  for(int idx=0; idx<5; idx++) {
    shared.wait_one();
    dump_clock("cp2");
  }
    
  std::this_thread::sleep_for(std::chrono::seconds(3));
  dump_clock("cp3");

  for(int idx=0; idx<15; idx++) {
    shared.wait_one();
    dump_clock("cp4");
  }

  prod.join();
    
  return 0;
}
