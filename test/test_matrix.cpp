#include <iostream>

#include "matrix.hpp"

int test_matrix_equal()
{
    int status = 0;

    QuadMatrix<3> m3x3;
    QuadMatrix<3> m3x3_2;

    std::cout << "Test equality" << std::endl;

    if (m3x3 != m3x3_2)
    {
        std::cout << "Test equality: Failed inequality test" << std::endl;
        status++;
    }

    if (m3x3 != m3x3_2.transpose())
    {
        std::cout << "Test equality: Failed transpose equality test" << std::endl;
        status++;
    }

    m3x3.set(1, 0, 5.0f);
    if (m3x3 == m3x3_2)
    {
        std::cout << "Test equality: Failed false equality test" << std::endl;
        status++;
    }

    return status;
}

int test_matrix_minor()
{
    int status = 0;
    QuadMatrix<2> m2x2_identity;
    QuadMatrix<3> m3x3;

    std::cout << "Test minor" << std::endl;

    std::cout << "Test 3x3 minor (0, 0)" << std::endl;
    QuadMatrix<2> minor00 = m3x3.minor(0, 0);
    if (minor00 != m2x2_identity) {
        std::cout << "Test 3x3 minor (0, 0) failed" << std::endl;
        status++;
    }

    return status;
}

int test_matrix_determinant()
{
    int status = 0;
    QuadMatrix<3> m3x3;
    QuadMatrix<4> m4x4;

    std::cout << "Test determinant" << std::endl;

    std::cout << "Test 3x3 determinant of identity" << std::endl;
    if (m3x3.determinant() != 1)
    {
        std::cout << "Test determinant: Failed identity test" << std::endl;
        status++;
    }

    std::cout << "Test 4x4 determinant of identity" << std::endl;
    if (m4x4.determinant() != 1)
    {
        std::cout << "Test determinant: Failed identity test" << std::endl;
        status++;
    }

    std::cout << "Test 4x4 determinant of matrix with low rank" << std::endl;
    m4x4.set(1, 1, 0.0f);
    if (m4x4.determinant() != 0)
    {
        std::cout << "Test determinant: Failed rank < 4 test" << std::endl;
        status++;
    }

    return status;
}

int test_matrix_inverse()
{
    int status = 0;
    QuadMatrix<3> m3x3;

    std::cout << "Test inverse" << std::endl;

    auto inverse = m3x3.inverse();
    if (inverse != m3x3)
    {
        status++;
    }

    return status;
}

int test_matrix()
{
    return test_matrix_equal() + test_matrix_minor() + test_matrix_determinant() + test_matrix_inverse();
}

int main(int argc, char *argv[])
{
    int status = test_matrix();
    if (status != 0)
    {
        std::cout << "Tests failed." << std::endl;
    }
    return status;
}