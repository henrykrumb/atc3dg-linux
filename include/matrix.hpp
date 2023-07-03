#pragma once

template <int N>
class QuadMatrix
{
public:
    QuadMatrix() { identity(); }
    QuadMatrix(const float (&data)[N][N]) { fromArray(data); }
    QuadMatrix(const QuadMatrix &other) { copy(other); }
    virtual ~QuadMatrix() {}

    void copy(const QuadMatrix &other);
    void fromArray(const float (&data)[N][N]);
    void toArray(float (&data)[N][N]) const;

    void identity();
    void set(const int i, const float value, const bool row_first=true);
    void set(const int i, const int j, const float value);
    float get(const int i, const int j) const;
    QuadMatrix multiply(const QuadMatrix &other);
    QuadMatrix multiply(const float scalar);

    QuadMatrix adjoint();
    QuadMatrix cofactor();
    QuadMatrix<N - 1> minor(const int strike_row, const int strike_column);
    float determinant();
    float trace();
    QuadMatrix inverse();
    QuadMatrix transpose();

    bool equals(const QuadMatrix& other) const;
    bool operator==(const QuadMatrix& other) { return equals(other); }
    bool operator!=(const QuadMatrix& other) { return !equals(other); }

    // TODO: overload operators

protected:
    float m_data[N][N];
};

#include "matrix.tpp"