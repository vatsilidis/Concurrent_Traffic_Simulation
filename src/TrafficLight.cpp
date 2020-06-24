#include <iostream>
#include <random>
#include "TrafficLight.h"


/* Implementation of class "MessageQueue" */

template <class T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
	std::unique_lock<std::mutex> lock(_mutex);
    _condition.wait(lock, [this]{ return  !_queue.empty();});
    T msg = std::move(_queue.front());
    _queue.pop_front();
	return msg;
}

template <class T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    std::lock_guard<std::mutex> lock(_mutex);
    _queue.emplace_back(msg);
    _condition.notify_one();
}



/* Implementation of class "TrafficLight" */
TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto msg = _queue.receive();
        if(msg == green){
            return;
        }
        
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}
void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}


double changeTime()
{
    typedef std::chrono::high_resolution_clock clock;
    // generate a random time from 4 to 6 seconds
    clock::time_point beginning = clock::now();
	clock::duration d = clock::now() - beginning;
    unsigned seed = d.count();
	std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(4, 6);
    return distribution(generator) * 1000;
}

void TrafficLight::cycleThroughPhases()
{
    int cycleDuration = changeTime();
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    lastUpdate = std::chrono::system_clock::now();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration)
        {
            // reset light duration
          	cycleDuration = changeTime(); 
            switch (_currentPhase)
            {
            case TrafficLightPhase::green:
                _currentPhase = TrafficLightPhase::red;
                break;
            case TrafficLightPhase::red:
                _currentPhase = TrafficLightPhase::green;
                break;
            }
			//std::cout << "Current Traffic Light Phase: " << _currentPhase << std::endl;
            _queue.send(std::move(_currentPhase));
          	lastUpdate = std::chrono::system_clock::now();
        }
    }
}