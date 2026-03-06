/**
 * @file VelocityEstimator.cpp
 * @brief Implementation of velocity estimation algorithms
 */

#include "VelocityEstimator.h"
#include <string.h>  // For memset

// ============================================================================
// EDGE-TIME VELOCITY ESTIMATOR IMPLEMENTATION
// ============================================================================

EdgeTimeVelocityEstimator::EdgeTimeVelocityEstimator()
    : countsPerRev_(0)
    , prevCount_(0)
    , prevEdgeUs_(0)
    , velocity_(0.0f)
    , zeroTimeoutMs_(50)
    , filterSize_(4)
    , filterIndex_(0)
    , filterCount_(0)
    , initialized_(false)
{
    memset(filterBuffer_, 0, sizeof(filterBuffer_));
}

void EdgeTimeVelocityEstimator::init(uint16_t countsPerRev) {
    countsPerRev_ = countsPerRev;
    reset();
}

void EdgeTimeVelocityEstimator::reset() {
    prevCount_ = 0;
    prevEdgeUs_ = 0;       // 0 = sentinel: no baseline captured yet
    initialized_ = false;
    velocity_ = 0.0f;
    filterIndex_ = 0;
    filterCount_ = 0;
    memset(filterBuffer_, 0, sizeof(filterBuffer_));
}

void EdgeTimeVelocityEstimator::update(uint32_t lastEdgeUs, int32_t currentCount) {
    // -----------------------------------------------------------------------
    // Timeout check: use micros() (current wall-clock time) vs lastEdgeUs
    // (timestamp of the most recent encoder edge).
    //
    // Bug that was here: old code did `lastEdgeUs - prevEdgeUs_` which
    // computes time *between* edges (= 1/frequency), not time *since* last
    // edge. At low speed the edge gap easily exceeds the 50ms timeout even
    // while the motor is moving → false zero velocity.
    //
    // lastEdgeUs == 0 means no edges have arrived since reset — skip check.
    // -----------------------------------------------------------------------
    if (lastEdgeUs != 0) {
        uint32_t timeSinceLastEdgeUs = micros() - lastEdgeUs;
        if (timeSinceLastEdgeUs > (uint32_t)zeroTimeoutMs_ * 1000UL) {
            // Motor has stopped — clear filter and reset baseline so the
            // next edge gives a clean start (Bug 3 fix: update prevEdgeUs_
            // so we don't get stuck firing the timeout on every call).
            velocity_ = 0.0f;
            filterIndex_ = 0;
            filterCount_ = 0;
            memset(filterBuffer_, 0, sizeof(filterBuffer_));
            initialized_ = false;
            prevCount_  = currentCount;
            prevEdgeUs_ = lastEdgeUs;
            return;
        }
    }

    // -----------------------------------------------------------------------
    // First call after reset or timeout: capture baseline, don't compute.
    // (Bug 2 fix: old reset() set prevEdgeUs_ = micros() and prevCount_ = 0,
    // causing a huge deltaCount spike and/or an immediate timeout on the
    // first real edge.)
    // -----------------------------------------------------------------------
    if (!initialized_) {
        prevCount_   = currentCount;
        prevEdgeUs_  = lastEdgeUs;
        initialized_ = (lastEdgeUs != 0);  // wait for at least one real edge
        return;
    }

    // No new edges since last update — keep previous velocity estimate
    int32_t deltaCount = currentCount - prevCount_;
    if (deltaCount == 0) {
        return;
    }

    // Time span covered by the new edges
    uint32_t deltaTimeUs = lastEdgeUs - prevEdgeUs_;
    if (deltaTimeUs == 0) {
        return;  // Shouldn't happen; guard against divide-by-zero
    }

    // Instantaneous velocity in ticks/second
    float instantVelocity = (float)deltaCount * 1000000.0f / (float)deltaTimeUs;

    addFilterSample(instantVelocity);
    velocity_ = getFilteredVelocity();

    prevCount_  = currentCount;
    prevEdgeUs_ = lastEdgeUs;
}

float EdgeTimeVelocityEstimator::getVelocity() const {
    return velocity_;
}

void EdgeTimeVelocityEstimator::setFilterSize(uint8_t size) {
    if (size < 2) size = 2;
    if (size > MAX_FILTER_SIZE) size = MAX_FILTER_SIZE;

    filterSize_ = size;

    // Reset filter if size changed
    filterIndex_ = 0;
    filterCount_ = 0;
    memset(filterBuffer_, 0, sizeof(filterBuffer_));
}

void EdgeTimeVelocityEstimator::setZeroTimeout(uint16_t timeoutMs) {
    zeroTimeoutMs_ = timeoutMs;
}

void EdgeTimeVelocityEstimator::addFilterSample(float sample) {
    filterBuffer_[filterIndex_] = sample;
    filterIndex_ = (filterIndex_ + 1) % filterSize_;

    if (filterCount_ < filterSize_) {
        filterCount_++;
    }
}

float EdgeTimeVelocityEstimator::getFilteredVelocity() const {
    if (filterCount_ == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < filterCount_; i++) {
        sum += filterBuffer_[i];
    }

    return sum / (float)filterCount_;
}

// ============================================================================
// PULSE-COUNT VELOCITY ESTIMATOR IMPLEMENTATION
// ============================================================================

PulseCountVelocityEstimator::PulseCountVelocityEstimator()
    : countsPerRev_(0)
    , windowStartCount_(0)
    , windowStartUs_(0)
    , velocity_(0.0f)
    , timeWindowUs_(20000)  // Default 20ms window
{
}

void PulseCountVelocityEstimator::init(uint16_t countsPerRev) {
    countsPerRev_ = countsPerRev;
    reset();
}

void PulseCountVelocityEstimator::reset() {
    windowStartCount_ = 0;
    windowStartUs_ = micros();
    velocity_ = 0.0f;
}

void PulseCountVelocityEstimator::update(uint32_t currentUs, int32_t currentCount) {
    // Check if time window has elapsed
    uint32_t deltaTimeUs = currentUs - windowStartUs_;

    if (deltaTimeUs < timeWindowUs_) {
        // Still accumulating counts in current window
        return;
    }

    // Time window elapsed - compute velocity
    int32_t deltaCount = currentCount - windowStartCount_;

    if (deltaTimeUs == 0) {
        // Avoid division by zero
        velocity_ = 0.0f;
    } else {
        // Compute velocity: ticks/second
        velocity_ = (float)deltaCount * 1000000.0f / (float)deltaTimeUs;
    }

    // Start new window
    windowStartCount_ = currentCount;
    windowStartUs_ = currentUs;
}

float PulseCountVelocityEstimator::getVelocity() const {
    return velocity_;
}

void PulseCountVelocityEstimator::setTimeWindow(uint16_t windowMs) {
    timeWindowUs_ = (uint32_t)windowMs * 1000UL;
}
