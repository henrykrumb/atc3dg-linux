#include <cmath>

template <int N>
void Vector<N>::fromArray(const float (&data)[N])
{
    for (int i = 0; i < N; i++)
    {
        m_data[i] = data[i];
    }
}

template <int N>
void Vector<N>::copy(const Vector<N> &other)
{
    for (int i = 0; i < N; i++)
    {
        m_data[i] = other.m_data[i];
    }
}

template <int N>
void Vector<N>::toArray(float (&data)[N])
{
    for (int i = 0; i < N; i++)
    {
        data[i] = m_data[i];
    }
}

template <int N>
float Vector<N>::get(const int i)
{
    // TODO assert wrong index
    return m_data[i];
}

template <int N>
void Vector<N>::set(const int i, const float value)
{
    m_data[i] = value;
}

template <int N>
float Vector<N>::length() const
{
    float l = 0;
    for (int i = 0; i < N; i++)
    {
        l += m_data[i] * m_data[i];
    }
    return std::sqrt(l);
}