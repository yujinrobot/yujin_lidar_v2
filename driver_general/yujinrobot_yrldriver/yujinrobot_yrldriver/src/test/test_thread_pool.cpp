#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>


//hj
#include <atomic>
#include <iostream>
class StopWatch
{
public:
    StopWatch()
        : _start(std::chrono::system_clock::now())
    {

    }

    void Start()
    {
        _start = std::chrono::system_clock::now();
    }

    int GetTimeElapsed()
    {
        _end = std::chrono::system_clock::now();
        std::chrono::milliseconds mill = std::chrono::duration_cast<std::chrono::milliseconds>(_end - _start);
        return mill.count();
    }

    int GetTimeElapsedInSec()
    {
        _end = std::chrono::system_clock::now();
        std::chrono::seconds secc = std::chrono::duration_cast<std::chrono::seconds>(_end - _start);
        //              std::chrono::duration<double> secc = _end - _start;
        return secc.count();
    }

    double GetCurrentTimeInSeconds()
    {
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        return std::chrono::duration<double>(now.time_since_epoch()).count();
    }

protected:
    std::chrono::time_point<std::chrono::system_clock> _start;
    std::chrono::time_point<std::chrono::system_clock> _end;
};

class YujinRobotYrlDriver
{
public:
    YujinRobotYrlDriver();
    ~YujinRobotYrlDriver();

    void StartThreads();
    void removeThreads();

    void threadedFunction1();
    void threadedFunction2();
    
    void getScanningData();
    void dataProcessing(int & val);

private:
    std::thread mFirstThread;
    std::thread mSecondThread;
    std::atomic <bool> mbRunThread;

    std::deque<int> mInputDataDeque;
    std::deque<int> mOutputDataDeque;
    std::mutex mInputDataLocker;
    std::mutex mOutputDataLocker;

    std::mutex m_1;
    std::mutex m_2;

    std::condition_variable cv_1;
    std::condition_variable cv_2;

public:
    bool mbStopGettingScanData;

};

YujinRobotYrlDriver::YujinRobotYrlDriver()
: mbRunThread(false)
, mbStopGettingScanData(false)
{
    StartThreads();
}

YujinRobotYrlDriver::~YujinRobotYrlDriver()
{
    removeThreads();
}

void YujinRobotYrlDriver::StartThreads()
{
    if (!mbRunThread)
    {
        mbRunThread = true;
        mFirstThread = std::thread(&YujinRobotYrlDriver::threadedFunction1, this);
        mSecondThread = std::thread(&YujinRobotYrlDriver::threadedFunction2, this);
    }
}

void YujinRobotYrlDriver::removeThreads()
{
    if (mbRunThread)
    {
        mbRunThread = false;

        cv_1.notify_all();
        cv_2.notify_all();

        if (mFirstThread.joinable())
        {
            mFirstThread.join();
        }

        if (mSecondThread.joinable())
        {
            mSecondThread.join();
        }
    }
}

void YujinRobotYrlDriver::threadedFunction1()//25hz
{
    StopWatch sw;
    sw.Start();
    while (1)
    {
        while(sw.GetTimeElapsed() < 40){}
        sw.Start();
        
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        std::unique_lock<std::mutex> lock(m_1);
        cv_1.wait(lock, [this]{return !mbStopGettingScanData || !mbRunThread;});
        if (!mbRunThread) {
            return;
        }

        getScanningData();
    }
}

void YujinRobotYrlDriver::getScanningData()
{ 
    static int i = 0;
    {
        std::lock_guard<std::mutex> lock(m_2);
        mInputDataDeque.push_back(i);
        //std::cout << "push value\n";
    }
    //std::cout << "Notifying... \n";
    cv_2.notify_all();
    i++;
}

void YujinRobotYrlDriver::threadedFunction2()//20hz
{
    StopWatch sw;
    sw.Start();
    while (1)
    {
        //while(sw.GetTimeElapsed() < 50){} // 20hz
        while(sw.GetTimeElapsed() < 70){} // < 15hz
        sw.Start();
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //std::this_thread::sleep_for(std::chrono::milliseconds(20));

        int val;
        {
            std::unique_lock<std::mutex> lock(m_2);
            //std::cout << "Waiting... \n";
            cv_2.wait(lock, [this]{return !mInputDataDeque.empty() || !mbRunThread;});
            if (!mbRunThread) {
                return;
            }
            //std::cout << "...finished waiting.\n";

            int sizeInputDataDeque (mInputDataDeque.size());

            if(sizeInputDataDeque >= 60) /// optimization for low computational power environment
            {
                std::cout << "clear mInputDataDeque :  " << sizeInputDataDeque << "\n";
                std::cout << "THERE IS A DELAY IN REAL-TIME INPUT DATA PROCESSING. PLEASE CHECK THE SYSTEM PERFORMANCE.\n";
                mInputDataDeque.clear();

                continue;
            }
            else if (sizeInputDataDeque >= 30) /// 30 data processing = data for 1 rotation
            {
                std::cout << "popfront mInputDataDeque :  " << sizeInputDataDeque << "\n";
                mInputDataDeque.pop_front();
                
                //continue;
            }

            //std::cout << "get value\n";
            val = mInputDataDeque.front();
            mInputDataDeque.pop_front();
        }

        dataProcessing(val);
    }
}

void YujinRobotYrlDriver::dataProcessing( int & val)
{
    std::cout << val << "\n";




    // mInputDataLocker.lock();
    // int sizeInputDataDeque (mInputDataDeque.size());

    // if(sizeInputDataDeque >= 60) /// optimization for low computational power environment
    // {
    //     std::cout << "clear mInputDataDeque :  " << sizeInputDataDeque << "\n";
    //     std::cout << "THERE IS A DELAY IN REAL-TIME INPUT DATA PROCESSING. PLEASE CHECK THE SYSTEM PERFORMANCE.\n";

    //     mInputDataDeque.clear();
    //     mInputDataLocker.unlock();
    //     return;
    // }
    // else if (sizeInputDataDeque >= 30) /// 30 data processing = data for 1 rotation
    // {
    //     std::cout << "popfront mInputDataDeque :  " << sizeInputDataDeque << "\n";

    //     mInputDataDeque.pop_front();
    //     mInputDataLocker.unlock();
    //     return;
    // }

    // val = mInputDataDeque.front();
    // mInputDataDeque.pop_front();
    // mInputDataLocker.unlock();
}

int main()
{
    YujinRobotYrlDriver* instance = new YujinRobotYrlDriver();

    while(1){}

    delete instance;
    return 0;
}

