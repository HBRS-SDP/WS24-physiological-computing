/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

/* ================================================================================
 * Copyright: (C) 2024, Yash Somaiya, 
 *       Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Authors: Yash Somaiya yash.somaiya@smail.inf.h-brs.de
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *      See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Manager/threadManager.h>

using namespace hriPhysio::Manager;


ThreadManager::ThreadManager() {
    
    //-- Ensure we've empty control containers.
    pool.clear();
    status.clear();

    //-- Set the state of running.
    running = true;
}


ThreadManager::~ThreadManager() {
    //-- Close everything up.
    this->close();
}


std::thread::id ThreadManager::addThread(std::function<void(void)> func, const bool start/*=true*/) {
    
    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- Spawn a thread with the provided function.
    pool.push_back( std::thread(func) );

    //-- Get this threads id, and set it's state to the input boolean.
    std::thread::id thread_id = pool[pool.size()-1].get_id();
    status[thread_id] = start;

    //-- Unlock the mutex and return.
    lock.unlock();

    //-- Return the thread id to the caller.
    return thread_id;
}


std::thread::id ThreadManager::addLoopThread(std::function<void(void)> func, const double period/*=0.0*/, const bool start/*=true*/) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- Spawn a thread with the looper wrapper, taking the provided function and the looping period.
    pool.push_back( std::thread(&ThreadManager::looperWrapper, this, func, period) );

    //-- Get this threads id, and set it's state to the input boolean.
    std::thread::id thread_id = pool[pool.size()-1].get_id();
    status[thread_id] = start;

    //-- Unlock the mutex and return.
    lock.unlock();

    //-- Return the thread id to the caller.
    return thread_id;
}


void ThreadManager::interruptThread(const std::thread::id thread_id) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- Try interrupting this thread.
    status[thread_id] = false;

    //-- Unlock the mutex and return.
    lock.unlock();

    return;
}


bool ThreadManager::getThreadStatus(const std::thread::id thread_id) {

    //TODO: Maybe don't lock here.. just return the status.

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- Get the status of this thread.
    bool thread_status = status[thread_id];

    //-- Unlock the mutex and return.
    lock.unlock();

    return thread_status;
}


bool ThreadManager::getManagerRunning() {
    return running;
}


void ThreadManager::start() {
    
    if (this->getManagerRunning()) {
        this->setThreadStatus(true);    
    }

    return;
}


void ThreadManager::stop() {
    this->setThreadStatus(false);
    return;
}


void ThreadManager::close() {
    
    //-- Stop all the threads.
    this->stop();

    //-- Tell the threads to stop running.
    this->running = false;

    //-- Join all the threads.
    for (std::size_t idx = 0; idx < pool.size(); ++idx) {
        pool[idx].join();
    }

    //-- Destroy all elements in the pool and running.
    pool.clear();
    status.clear();

    return;
}


void ThreadManager::wait() {

    //-- Wait for the thread manager to stop.
    while (this->getManagerRunning()) {
        this->sleepThread(0.5);
    }

    return;
}


void ThreadManager::sleepThread(const double seconds) {

    std::this_thread::sleep_for(
        std::chrono::duration<double>( seconds )
    );

    return;
}


void ThreadManager::setThreadStatus(const bool thread_status) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- Get an iterator to the start of the map.
    std::map< std::thread::id, bool >::iterator it = status.begin();
    while (it != status.end()) {
        it->second = thread_status;
        ++it;
    }

    //-- Unlock the mutex and return.
    lock.unlock();

    return;
}


void ThreadManager::looperWrapper(std::function<void(void)> func, const double period) {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    //-- Loop until it's time to shutdown.
    while (this->getManagerRunning()) {

        //-- Get the clock time for now.
        auto start = std::chrono::system_clock::now();

        //-- Call the provided function if this thread is enabled.
        if (this->getThreadStatus(thread_id)) {
            func();
        }

        //-- Sleep the thread for the remaining time of the period.
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> dur = end - start;
        double diff = period - dur.count();

        diff = (diff > 0.0) ? diff : 0.0;
        
        this->sleepThread(diff);

        //auto loop_time = std::chrono::system_clock::now();
        //std::chrono::duration<double> loop_dur = loop_time - start;
        //std::cout << "[DEBUG] Thread id ("<< thread_id << ") executed in " << loop_dur.count() << " seconds." << std::endl;
    }
}
