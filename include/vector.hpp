#pragma once

template <int N>
class Vector
{
public:
    Vector() {}
    Vector(const float (&data)[N]) { fromArray(data); }
    Vector(const Vector& other) { copy(other); }
    virtual ~Vector() {}

    void fromArray(const float (&data)[N]);
    void copy(const Vector<N> &other);
    void toArray(float (&data)[N]);

    float get(const int i);
    void set(const int i, const float value);
    float length() const;

protected:
    float m_data[N];
};


#include "vector.tpp"