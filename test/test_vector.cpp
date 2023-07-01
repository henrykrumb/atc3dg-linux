#include <iostream>

#include "vector.hpp"

int test_vector_length()
{
    int status = 0;

    Vector<3> v3({1.0f, 0.0f, 0.0f});
    if (v3.length() != 1.0f)
    {
        status++;
    }

    Vector<4> v4({1.0f, 1.0f, 1.0f, 1.0f});
    if (v4.length() != 2.0f)
    {
        status++;
    }
    return status;
}

int test_vector()
{
    return test_vector_length();
}

int main(int argc, char *argv[])
{
    int status = test_vector();
    if (status != 0)
    {
        std::cout << "Tests failed." << std::endl;
    }
    return status;
}