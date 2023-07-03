template <int N>
void QuadMatrix<N>::copy(const QuadMatrix<N> &other)
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            m_data[i][j] = other.m_data[i][j];
        }
    }
}

template <int N>
void QuadMatrix<N>::fromArray(const float (&data)[N][N])
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            m_data[i][j] = data[i][j];
        }
    }
}

template <int N>
void QuadMatrix<N>::toArray(float (&data)[N][N]) const
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            data[i][j] = m_data[i][j];
        }
    }
}

template <int N>
void QuadMatrix<N>::identity()
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            m_data[i][j] = 0.0f;
            if (i == j)
            {
                m_data[i][j] = 1.0f;
            }
        }
    }
}

template <int N>
void QuadMatrix<N>::set(const int i, const float value, const bool row_first)
{
    // TODO assert 0 <= i < N * N
    if (row_first)
    {
        m_data[i % N][i / N] = value;
    }
    else
    {
        m_data[i / N][i % N] = value;
    }
}

template <int N>
void QuadMatrix<N>::set(const int i, const int j, const float value)
{
    // TODO assert i , j
    m_data[i][j] = value;
}

template <int N>
float QuadMatrix<N>::get(const int i, const int j) const
{
    // TODO assert i , j
    return m_data[i][j];
}

template <int N>
QuadMatrix<N> QuadMatrix<N>::multiply(const QuadMatrix<N> &other)
{
    QuadMatrix<N> result;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            for (int k = 0; k < N; k++)
            {
                result.m_data[i][k] = m_data[i][j] * other.m_data[j][k];
            }
        }
    }
    return result;
}

template <int N>
QuadMatrix<N> QuadMatrix<N>::multiply(const float scalar)
{
    QuadMatrix<N> result;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            result.m_data[i][j] = m_data[i][j] * scalar;
        }
    }
    return result;
}

template <int N>
QuadMatrix<N> QuadMatrix<N>::adjoint()
{
    return cofactor().transpose();
}

template <int N>
QuadMatrix<N> QuadMatrix<N>::cofactor()
{
    QuadMatrix<N> C;

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            float det = minor(i, j).determinant();
            if ((i + j) % 2 != 0)
            {
                det = -det;
            }
            C.m_data[i][j] = det;
        }
    }

    return C;
}

template <>
float QuadMatrix<2>::determinant()
{
    float det = 0.0f;
    det += m_data[0][0] * m_data[1][1] - m_data[0][1] * m_data[1][0];
    return det;
}

template <>
float QuadMatrix<3>::determinant()
{
    float det = 0.0f;
    det += m_data[0][0] * m_data[1][1] * m_data[2][2];
    det += m_data[0][1] * m_data[1][2] * m_data[2][0];
    det += m_data[0][2] * m_data[1][0] * m_data[2][1];
    return det;
}

template <int N>
QuadMatrix<N - 1> QuadMatrix<N>::minor(const int strike_row, const int strike_column)
{
    // TODO assert N > 2
    int mi = 0;
    int mj = 0;
    QuadMatrix<N - 1> minor_matrix;
    for (int i = 0; i < N; i++)
    {
        if (i == strike_row)
        {
            continue;
        }
        for (int j = 0; j < N; j++)
        {
            if (j == strike_column)
            {
                continue;
            }
            minor_matrix.set(mi, mj, m_data[i][j]);
            mj++;
        }
        mi++;
        mj = 0;
    }
    return minor_matrix;
}

template <int N>
float QuadMatrix<N>::determinant()
{
    // TODO assert N >= 4, if template specification failed at some point
    float det = 0.0f;
    for (int j = 0; j < N; j++)
    {
        // choose third row, as we're likely to have many zeros in there
        // if we're dealing with homogeneous 4x4 transforms
        int i = 3;
        if (m_data[i][j] == 0)
        {
            continue;
        }
        int minor_determinant = minor(i, j).determinant();
        if ((i + j) % 2 != 0)
        {
            minor_determinant = -minor_determinant;
        }
        det += minor_determinant * m_data[i][j];
    }
    return det;
}

template <int N>
float QuadMatrix<N>::trace()
{
    float result = 0;
    for (int i = 0; i < N; i++)
    {
        result += m_data[i][i];
    }
    return result;
}

template <int N>
QuadMatrix<N> QuadMatrix<N>::inverse()
{
    return adjoint().multiply(1 / determinant());
}

template <int N>
QuadMatrix<N> QuadMatrix<N>::transpose()
{
    QuadMatrix<N> T;

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            T.m_data[i][j] = m_data[j][i];
        }
    }

    return T;
}

template <int N>
bool QuadMatrix<N>::equals(const QuadMatrix<N> &other) const
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (m_data[i][j] != other.m_data[i][j])
            {
                return false;
            }
        }
    }
    return true;
}