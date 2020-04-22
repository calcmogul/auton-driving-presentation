// Copyright (c) 2020 FIRST. All Rights Reserved.

#pragma once

class Encoder {
public:
    Encoder(int channelA, int channelB) {}

    void SetDistancePerPulse(double distancePerPulse) {
        m_distancePerPulse = distancePerPulse;
    }

    double GetDistance() const { return m_count * m_distancePerPulse; }

    void SetSamplesToAverage(int samplesToAverage) {}

    double GetRate() const { return m_rate * m_distancePerPulse; }

    void SetRate(double rate) { m_rate = rate; }

    void SetCount(double count) { m_count = count; }

private:
    double m_count = 0.0;
    double m_rate = 0.0;
    double m_distancePerPulse = 1.0;
};
