'''
This script is used to generate a C++ header file from a given template.

The header file should be structured as follows:

    #pragma once
    #include <iostream.h>

    float f{i} = (std::vector<float> x, std::vector<float> u) 
    {
        return {function};
    }

Then once all the aij, and bkl functions are written, then this script will generate the functions get_A() and get_B() for arbitrary NS and NC. and i, j, k, l as follows:


// Create a function that returns a Matrixxf of size (NS, NS)

Eigen::MatrixXf get_A() 
{
    std::map<std::string, std::function<int()>> functions = { {"a11", a11}, {"a12", a12}, {"a21", a21}, {"a22", a22} };

    Eigen::MatrixXf A(NS, NS);

    for (int i = 1; i <= NS; i++)
    {
        for (int j = 1; j <= NS; j++)
        {
            std::string function_name = "a" + std::to_string(i) + std::to_string(j);
            auto function = functions[function_name];
            A(i-1, j-1) = function();
        }
    }

    return A;
}

Eigen::MatrixXf get_B()
{
    std::map<std::string, std::function<int()>> functions = { {"b11", b11}, {"b12", b12}, {"b21", b21}, {"b22", b22} };

    Eigen::MatrixXf B(NS, NC);

    for (int i = 1; i <= NS; i++)
    {
        for (int j = 1; j <= NC; j++)
        {
            std::string function_name = "b" + std::to_string(i) + std::to_string(j);
            auto function = functions[function_name];
            B(i-1, j-1) = function();
        }
    }

    return B;
}
    



The header file will be created (if it doesn't exist) otherwise it will first delete it and create it again
by this script will have the name 'jacobian.h' and will be located in ../src/ relative
to the location of this script.

'''

#include <iostream>
#include <functional>
#include <map>

#define NS 2
#define NC 5



import sys
import os
import re

from dynamic_model import DynamicModel
import casadi as ca
import casadi.tools as ct

def main():
    dynamic_model = DynamicModel()
    functions = dynamic_model.compute_jacobian()

    for i in range(0, len(functions)):
        print(f'f[{i}] = {functions[i]}')


    # Check if relative path ../src/jacobian.h exists. If not, create it, otherwise delete it and create it again.
    if not os.path.exists(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h')):
        open(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'), 'w').close()
    else:
        os.remove(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'))
        open(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'), 'w').close()

    # Open the header file
    with open(os.path.join(os.path.dirname(__file__), '..', 'src', 'jacobian.h'), 'a') as f:
        # Write the header
        f.write('// This file is auto generated using the script \'generate_jacobian.py\'. Please do not edit. \r')
        f.write('#include <iostream.h> \r \r')

        # Given the template in the comment above write the file
        for i in range(0, len(functions)):
            f.write('float f' + str(i + 1) + ' = (std::vector<float> x, std::vector<float> u) \r')
            f.write('{ \r')
            f.write('    return ' + functions[i-1] + '; \r')
            f.write('} \r \r')

if __name__ == '__main__':
    main()


